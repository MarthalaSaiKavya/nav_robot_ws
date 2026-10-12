"""
frontier_explorer - autonomous exploration on top of SLAM + Nav2 (Phase 8).

    ros2 launch my_bot_bringup phase8_exploration.launch.py explore_strategy:=information_gain

LOOP (runs in a worker thread, see common/ros_utils.py for why):
   +-> read latest /map from SLAM
   |   find frontiers  -> drop blacklisted ones -> score with the chosen STRATEGY
   |   none left (several times in a row)?  -> DONE
   |   send the best frontier to Nav2 (NavigateToPose action)
   |   while driving, every 1/replan_rate s:
   |       * target frontier disappeared (we can already see behind it) -> pick next
   |       * no progress for progress_timeout s -> cancel + BLACKLIST that spot
   |       * Nav2 reports ABORTED (unreachable)  -> BLACKLIST
   +---------------------------------------------+
   DONE: drive back to start (optional) -> SAVE the map (+ pose graph) -> write a json
         summary -> exit (the launch file is set to shut the stage down when we exit).

MODES (parameter `strategy`):
   nearest | largest | utility | information_gain | random   -> this node drives
   external  -> another explorer (e.g. explore_lite) drives; this node only WATCHES the
                frontiers and does the "done -> return -> save map -> exit" part.
                "Done" = no frontiers left, OR the robot has not moved for
                external_idle_timeout s (the other explorer gave up).
"""
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from ..common.map_io import grid_from_msg
from ..common.nav2_actions import Nav2Actions
from ..common.ros_utils import ignore_sigint, lookup_pose_2d, make_run_dir, write_json
from ..mapping.map_saving import save_map_with_fallbacks, serialize_pose_graph
from .frontiers import STRATEGIES, filter_blacklist, find_frontiers, score_frontiers


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        p = self.declare_parameter
        self.strategy = p('strategy', 'utility').value
        self.map_topic = p('map_topic', '/map').value
        self.global_frame = p('global_frame', 'map').value
        self.robot_frame = p('robot_frame', 'base_footprint').value
        self.min_frontier_size = p('min_frontier_size', 0.4).value
        self.max_frontier_length = p('max_frontier_length', 3.0).value   # 0 = never split
        self.min_clearance = p('min_goal_clearance', 0.30).value
        self.gain_radius = p('gain_radius', 2.0).value
        self.potential_scale = p('potential_scale', 3.0).value
        self.gain_scale = p('gain_scale', 1.0).value
        self.use_path_distance = p('use_path_distance', False).value
        self.path_candidates = p('path_distance_candidates', 4).value
        self.replan_rate = p('replan_rate', 1.0).value
        self.progress_timeout = p('progress_timeout', 25.0).value
        self.blacklist_radius = p('blacklist_radius', 0.6).value
        self.min_goal_distance = p('min_goal_distance', 0.4).value
        self.completion_patience = p('completion_patience', 4).value
        self.max_duration = p('max_duration', 900.0).value
        self.initial_spin = p('initial_spin', True).value
        self.return_to_start = p('return_to_start', True).value
        self.save_map = p('save_map', True).value
        self.map_name = p('map_name', 'explored_map').value
        self.run_dir = make_run_dir(p('run_dir', '').value)
        self.save_methods = list(p('save_methods', ['nav2', 'slam_toolbox', 'python']).value)
        self.serialize = p('serialize_pose_graph', True).value
        self.exit_on_done = p('exit_on_done', True).value
        self.external_idle_timeout = p('external_idle_timeout', 60.0).value
        seed = p('random_seed', 0).value
        self.rng = np.random.default_rng(seed if seed else None)
        if self.strategy not in STRATEGIES + ('external',):
            raise ValueError(f'strategy must be one of {STRATEGIES + ("external",)}')

        self.cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = Nav2Actions(self, self.cb)
        self.map_msg = None
        self._cached_msg, self._cached_frontiers = None, []
        self.map_lock = threading.Lock()
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, map_qos, callback_group=self.cb)
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 1)
        self.status_pub = self.create_publisher(String, '/exploration/status', 10)
        self.explore_lite_resume = self.create_publisher(Bool, '/explore/resume', 1)

        self.blacklist = []
        self.reached_goals = []
        self.stats = {'strategy': self.strategy, 'goals_sent': 0, 'goals_reached': 0,
                      'targets_explored_early': 0, 'goals_failed': 0,
                      'blacklisted': 0, 'distance_travelled_m': 0.0}
        self.finished = threading.Event()
        self.worker = threading.Thread(target=self._run_safe, daemon=True)
        self.worker.start()

    # ---------------------------------------------------------------- helpers
    def _on_map(self, msg):
        with self.map_lock:
            self.map_msg = msg

    def _status(self, state, **extra):
        msg = dict(state=state, **extra)
        self.status_pub.publish(String(data=json.dumps(msg)))
        self.get_logger().info(f'[explore] {state} {extra if extra else ""}')

    def _robot(self):
        return lookup_pose_2d(self.tf_buffer, self.global_frame, self.robot_frame)

    def _sleep(self, sec):
        time.sleep(sec)

    def _frontiers(self, robot):
        with self.map_lock:
            msg = self.map_msg
        if msg is None:
            return []
        if msg is not self._cached_msg:
            # Frontier detection is the expensive part; SLAM publishes a new map only every
            # few seconds, so detect once per map and reuse the result in between.
            grid, res, origin = grid_from_msg(msg)
            self._cached_frontiers = find_frontiers(grid, res, origin, self.min_frontier_size,
                                                    self.min_clearance, self.gain_radius,
                                                    max_frontier_length_m=self.max_frontier_length)
            self._cached_msg = msg
        fr = filter_blacklist(list(self._cached_frontiers), self.blacklist, self.blacklist_radius)
        if not fr:
            return []
        distances = None
        if self.use_path_distance and self.strategy != 'external':
            # Ask Nav2's planner for the REAL driving distance of the closest few candidates.
            fr.sort(key=lambda f: math.hypot(f.goal[0] - robot[0], f.goal[1] - robot[1]))
            fr = fr[:self.path_candidates]
            distances = []
            for f in fr:
                d = self.nav.path_length(f.goal[0], f.goal[1])
                distances.append(d if d is not None else 1e6)   # unreachable -> very far
        strategy = 'nearest' if self.strategy == 'external' else self.strategy
        scored = score_frontiers(fr, robot[:2], strategy, self.potential_scale, self.gain_scale,
                                 self.rng, distances)
        # A "frontier" right under the robot is usually a sensor artefact (blind spot), skip it.
        return [f for f in scored
                if math.hypot(f.goal[0] - robot[0], f.goal[1] - robot[1]) >= self.min_goal_distance]

    def _publish_markers(self, frontiers, target=None):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for i, f in enumerate(frontiers):
            pts = Marker()
            pts.header.frame_id = self.global_frame
            pts.header.stamp = stamp
            pts.ns, pts.id, pts.type = 'frontier_cells', i, Marker.POINTS
            pts.scale.x = pts.scale.y = 0.05
            best = (i == 0)
            pts.color.r, pts.color.g, pts.color.b, pts.color.a = (0.1, 1.0, 0.1, 1.0) if best else (0.2, 0.6, 1.0, 0.8)
            with self.map_lock:
                info = self.map_msg.info if self.map_msg else None
            if info is None:
                continue
            for r, c in f.cells[::2]:
                pts.points.append(Point(x=info.origin.position.x + (c + 0.5) * info.resolution,
                                        y=info.origin.position.y + (r + 0.5) * info.resolution, z=0.05))
            arr.markers.append(pts)
            goal = Marker()
            goal.header = pts.header
            goal.ns, goal.id, goal.type = 'frontier_goals', i, Marker.SPHERE
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = f.goal[0], f.goal[1], 0.1
            goal.pose.orientation.w = 1.0
            goal.scale.x = goal.scale.y = goal.scale.z = 0.25 if best else 0.15
            goal.color = pts.color
            arr.markers.append(goal)
            txt = Marker()
            txt.header = pts.header
            txt.ns, txt.id, txt.type = 'frontier_scores', i, Marker.TEXT_VIEW_FACING
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = f.goal[0], f.goal[1], 0.45
            txt.scale.z = 0.18
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f'{f.score:.2f}'
            arr.markers.append(txt)
        for j, (bx, by) in enumerate(self.blacklist):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.global_frame, stamp
            m.ns, m.id, m.type = 'blacklist', j, Marker.CYLINDER
            m.pose.position.x, m.pose.position.y = bx, by
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2 * self.blacklist_radius
            m.scale.z = 0.02
            m.color.r, m.color.a = 1.0, 0.4
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    # ---------------------------------------------------------------- main logic
    def _run_safe(self):
        try:
            self._run()
        except Exception as e:  # make sure a bug ends the run visibly instead of hanging
            self.get_logger().error(f'explorer crashed: {e!r}')
            self.stats['error'] = repr(e)
            self._finish(success=False)

    def _wait_ready(self):
        self._status('WAITING', what='map, TF and Nav2')
        t0 = time.time()
        while rclpy.ok():
            if self.map_msg is not None and self._robot() is not None:
                break
            if time.time() - t0 > 120:
                raise RuntimeError('no /map or TF map->base_footprint after 120 s (is SLAM running?)')
            self._sleep(0.5)
        # The action servers appear as soon as Nav2 starts, but goals are REJECTED until the
        # lifecycle manager has activated everything -> wait for 'active', not just 'exists'.
        if not self.nav.wait_until_active(timeout=180):
            raise RuntimeError('Nav2 did not become active within 180 s (is navigation launched?)')

    def _run(self):
        self._wait_ready()
        start = self._robot()
        self.stats['start_pose'] = start
        self.t_start = time.time()
        self._status('STARTED', strategy=self.strategy, start=[round(v, 2) for v in start])
        if self.initial_spin and self.strategy != 'external':
            # Look all around first: the map grows a lot "for free" and frontiers become meaningful.
            self._status('INITIAL_SPIN')
            self.nav.spin_in_place(6.28, 30)

        empty = 0
        rejections = 0
        last_pose = start
        idle_pose, idle_since = start, time.time()      # only used in 'external' mode
        while rclpy.ok():
            if time.time() - self.t_start > self.max_duration:
                self._status('TIMEOUT', seconds=self.max_duration)
                break
            robot = self._robot()
            if robot is None:
                self._sleep(0.5)
                continue
            frontiers = self._frontiers(robot)
            self._publish_markers(frontiers)
            if not frontiers:
                empty += 1
                self._status('NO_FRONTIERS', count=empty, patience=self.completion_patience)
                if empty >= self.completion_patience:
                    break
                self._sleep(2.0)            # the map may still be updating - check again
                continue
            empty = 0
            if self.strategy == 'external':
                self.stats['distance_travelled_m'] += math.hypot(robot[0] - last_pose[0], robot[1] - last_pose[1])
                last_pose = robot
                # The other explorer may give up while WE still see (unreachable) frontiers,
                # e.g. explore_lite: "All frontiers traversed/tried out, stopping." -> if the
                # robot has not moved for external_idle_timeout s, consider exploration over.
                if math.hypot(robot[0] - idle_pose[0], robot[1] - idle_pose[1]) > 0.2 or \
                        abs(math.remainder(robot[2] - idle_pose[2], 2 * math.pi)) > 0.5:
                    idle_pose, idle_since = robot, time.time()
                elif time.time() - idle_since > self.external_idle_timeout:
                    self._status('EXTERNAL_EXPLORER_IDLE', seconds=self.external_idle_timeout)
                    break
                self._sleep(1.0 / self.replan_rate)
                continue
            target = frontiers[0]
            if any(math.hypot(target.goal[0] - gx, target.goal[1] - gy) < 0.5 for gx, gy in self.reached_goals):
                # We already stood here and it is STILL a frontier (e.g. an area the LiDAR cannot
                # see into). Going again would loop forever -> give up on this spot.
                self._blacklist(target.goal, 'reached before but still a frontier')
                continue
            yaw = math.atan2(target.goal[1] - robot[1], target.goal[0] - robot[0])
            self._status('GOAL', x=round(target.goal[0], 2), y=round(target.goal[1], 2),
                         score=round(target.score, 3), size_m=round(target.size_m, 2),
                         distance=round(target.distance, 2), candidates=len(frontiers))
            self.stats['goals_sent'] += 1
            handle = self.nav.navigate(target.goal[0], target.goal[1], yaw)
            if not handle.accepted:
                # A REJECTED goal says nothing about the frontier: Nav2 was busy/not ready
                # (e.g. still activating, or a costmap reset). Retry a few times before giving up.
                rejections += 1
                self._status('GOAL_REJECTED', attempt=rejections)
                if rejections >= 5:
                    self._blacklist(target.goal, 'goal rejected 5x')
                    rejections = 0
                self._sleep(2.0)
                continue
            rejections = 0
            outcome = self._monitor(handle, target)
            pose = self._robot() or last_pose
            self.stats['distance_travelled_m'] += math.hypot(pose[0] - last_pose[0], pose[1] - last_pose[1])
            last_pose = pose
            if outcome == 'reached':
                self.stats['goals_reached'] += 1
                self.reached_goals.append(target.goal)
            elif outcome == 'explored':
                # The usual case: we saw past the frontier before arriving -> next target.
                self.stats['targets_explored_early'] += 1
            elif outcome in ('failed', 'stuck'):
                self.stats['goals_failed'] += 1
                self._blacklist(target.goal, outcome)

        self._finish(success=True, start=start)

    def _monitor(self, handle, target):
        """Watch a running goal. Returns 'reached' | 'explored' | 'failed' | 'stuck'."""
        best_dist = math.inf
        last_progress = time.time()
        while rclpy.ok():
            if handle.wait(1.0 / self.replan_rate):
                return 'reached' if handle.succeeded else 'failed'
            robot = self._robot()
            if robot is None:
                continue
            d = math.hypot(target.goal[0] - robot[0], target.goal[1] - robot[1])
            if d < best_dist - 0.10:
                best_dist, last_progress = d, time.time()
            elif time.time() - last_progress > self.progress_timeout:
                self._status('NO_PROGRESS', seconds=self.progress_timeout)
                handle.cancel()
                return 'stuck'
            # Is our target frontier still a frontier? (Often we see past it before arriving.)
            frontiers = self._frontiers(robot)
            self._publish_markers(frontiers)
            still_there = any(math.hypot(f.goal[0] - target.goal[0], f.goal[1] - target.goal[1]) < 0.75
                              for f in frontiers)
            if not still_there:
                self._status('TARGET_EXPLORED')
                handle.cancel()
                return 'explored'
        handle.cancel()
        return 'failed'

    def _blacklist(self, goal, why):
        self.blacklist.append(goal)
        self.stats['blacklisted'] += 1
        self._status('BLACKLIST', x=round(goal[0], 2), y=round(goal[1], 2), reason=why)

    def _finish(self, success, start=None):
        if self.finished.is_set():
            return
        self.stats['duration_s'] = round(time.time() - getattr(self, 't_start', time.time()), 1)
        if self.strategy == 'external':
            self.explore_lite_resume.publish(Bool(data=False))     # tell explore_lite to stop
            # explore_lite stops by cancelling ALL navigate_to_pose goals - give it a moment,
            # otherwise it may cancel our "return home" goal that we are about to send.
            self._sleep(2.0)
        if success and self.return_to_start and start is not None:
            self._status('RETURNING_HOME')
            if not self.nav.wait_for_servers(timeout=10):
                self.get_logger().warn('cannot return home: Nav2 not available')
            else:
                # Up to 3 tries: a single ABORT (e.g. a TF hiccup while SLAM optimises the
                # graph) should not leave the robot in a corner.
                for attempt in range(1, 4):
                    h = self.nav.navigate(start[0], start[1], start[2])
                    if not h.wait(180):
                        h.cancel()
                    self.stats['returned_home'] = h.succeeded
                    if h.succeeded:
                        break
                    self._status('RETURN_HOME_FAILED', attempt=attempt, status=h.status_name)
                    self._sleep(2.0)
        if self.save_map:
            self._status('SAVING_MAP')
            base = os.path.join(self.run_dir, self.map_name)
            method = save_map_with_fallbacks(self, base, self.save_methods, self.map_topic)
            self.stats['map_yaml'] = base + '.yaml' if method else None
            self.stats['map_save_method'] = method
            if self.serialize:
                self.stats['pose_graph_saved'] = serialize_pose_graph(self, base)
        self.stats['success'] = bool(success and (not self.save_map or self.stats.get('map_yaml')))
        write_json(os.path.join(self.run_dir, 'exploration_result.json'), self.stats)
        self._status('DONE', **{k: v for k, v in self.stats.items() if not isinstance(v, (list, tuple))})
        self.finished.set()
        if self.exit_on_done:
            # Give the DONE message time to go out, then stop spinning -> process exits.
            time.sleep(1.0)
            rclpy.try_shutdown()


def main(args=None):
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    rclpy.init(args=args)
    node = FrontierExplorer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ignore_sigint()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
