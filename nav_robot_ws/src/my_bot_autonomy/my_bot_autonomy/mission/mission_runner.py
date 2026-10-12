"""
mission_runner - executes a queued multi-goal mission with Nav2 (Phase 7).

    ros2 launch my_bot_bringup phase7_mission.launch.py mission_method:=follow_waypoints

FOUR WAYS TO SEND GOALS TO NAV2 (parameter `method`) - same mission, different technique:

  navigate_to_pose   (default) nav2_simple_commander.BasicNavigator.goToPose() per goal.
                     One goal at a time -> per-goal timeout, retries, events. Most control.
  follow_waypoints   BasicNavigator.followWaypoints(): hands the WHOLE list to Nav2's
                     waypoint_follower server, which visits them in order (and can run a
                     task at each one). Least code, least control.
  through_poses      BasicNavigator.goThroughPoses(): ONE continuous path through all poses,
                     the robot does not stop at intermediate poses. Smoothest, but one
                     unreachable pose fails the whole task.
  raw_action         Same as navigate_to_pose but with a plain rclpy ActionClient
                     (common/nav2_actions.py) - shows what BasicNavigator hides.

INITIAL POSE (parameter `initial_pose_mode`), i.e. how AMCL learns where we start:
  topic   publish the mission's ROUGH GUESS on /initialpose (default; like RViz "2D Pose Estimate")
  params  AMCL already got it from its parameters (set_initial_pose: true) -> do nothing
  global  call /reinitialize_global_localization: particles everywhere, then spin to converge
  none    another localizer (slam_toolbox / learning PF) handles it

EVENTS: publishes JSON strings on /mission/events (goal_started, goal_finished, ...)
which scenario_director uses to trigger obstacles. Results -> <run_dir>/mission_results.csv/json
"""
import csv
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from ..common.geometry import normalize_angle
from ..common.nav2_actions import Nav2Actions
from ..common.ros_utils import (call_service, ignore_sigint, lookup_pose_2d, make_initial_pose, make_pose_stamped,
                                make_run_dir, spin_in_background, write_json, yaw_of)
from .mission_io import load_mission

METHODS = ('navigate_to_pose', 'follow_waypoints', 'through_poses', 'raw_action')


class MissionNode(Node):
    """Everything EXCEPT driving: parameters, events, markers, TF, ground truth.
    Spun in a background thread; BasicNavigator (a separate node) spins itself."""

    def __init__(self):
        super().__init__('mission_runner')
        p = self.declare_parameter
        self.mission_file = p('mission_file', 'demo_mission.yaml').value
        self.method = p('method', 'navigate_to_pose').value
        self.initial_pose_mode = p('initial_pose_mode', 'topic').value
        self.pre_mission_spin = p('pre_mission_spin', True).value
        self.behavior_tree = p('behavior_tree', '').value
        self.run_dir = make_run_dir(p('run_dir', '').value)
        self.exit_on_done = p('exit_on_done', True).value
        self.start_delay = p('start_delay', 2.0).value
        if self.method not in METHODS:
            raise ValueError(f'method must be one of {METHODS}')

        self.events_pub = self.create_publisher(String, '/mission/events', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mission/markers', 1)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.truth = None
        self.truth_distance = 0.0
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 10)
        amcl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.amcl_cov = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, amcl_qos)
        self.global_loc = self.create_client(Empty, '/reinitialize_global_localization')
        self.bt_state = self.create_client(GetState, '/bt_navigator/get_state')
        self.actions = Nav2Actions(self)

    def _on_truth(self, msg):
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.truth is not None:
            self.truth_distance += math.hypot(pose[0] - self.truth[0], pose[1] - self.truth[1])
        self.truth = pose

    def _on_amcl(self, msg):
        c = msg.pose.covariance
        self.amcl_cov = (c[0], c[7], c[35])

    def event(self, event_name, **data):
        self.events_pub.publish(String(data=json.dumps(dict(event=event_name, **data))))
        self.get_logger().info(f"[mission] {event_name} {data}")

    def estimate(self):
        return lookup_pose_2d(self.tf_buffer, 'map', 'base_footprint')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def markers(self, goals, status):
        arr = MarkerArray()
        colors = {'pending': (0.6, 0.6, 0.6), 'active': (1.0, 0.85, 0.0),
                  'SUCCEEDED': (0.1, 0.9, 0.1), 'FAILED': (0.95, 0.1, 0.1)}
        for g in goals:
            st = status.get(g['index'], 'pending')
            col = colors.get(st, colors['FAILED'])
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type = 'goals', g['index'], Marker.ARROW
            m.pose = make_pose_stamped(g['x'], g['y'], g['yaw']).pose
            m.scale.x, m.scale.y, m.scale.z = 0.5, 0.1, 0.1
            m.color.r, m.color.g, m.color.b, m.color.a = (*col, 1.0)
            t = Marker()
            t.header.frame_id = 'map'
            t.ns, t.id, t.type = 'goal_names', g['index'], Marker.TEXT_VIEW_FACING
            t.pose.position.x, t.pose.position.y, t.pose.position.z = g['x'], g['y'], 0.5
            t.scale.z = 0.25
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f"{g['index']}: {g['name']} [{st}]"
            arr.markers += [m, t]
        self.marker_pub.publish(arr)


class MissionRunner:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()
        self.mission = load_mission(node.mission_file)
        self.status = {}
        self.rows = []
        self.nav = None
        if node.method != 'raw_action':
            from nav2_simple_commander.robot_navigator import BasicNavigator
            self.nav = BasicNavigator(node_name='mission_basic_navigator')

    # ----------------------------------------------------------- preparation
    def wait_for_nav2(self, timeout=180.0):
        self.log.info('waiting for Nav2 (bt_navigator active) ...')
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            res = call_service(self.node.bt_state, GetState.Request(), timeout_sec=2.0, wait_service_sec=2.0)
            if res is not None and res.current_state.label == 'active':
                return True
            time.sleep(1.0)
        return False

    def set_initial_pose(self, timeout=180.0):
        """Give the localizer its initial pose, then wait until TF map -> base_footprint exists.

        ORDER MATTERS: Nav2's global costmap refuses to activate until map -> base_footprint
        exists, and AMCL publishes that only AFTER it got an initial pose. So we set the pose
        FIRST and wait for Nav2 SECOND (BasicNavigator.waitUntilNav2Active does the same).
        AMCL ignores /initialpose until it is active, so we simply re-publish every second
        until the transform appears."""
        mode = self.node.initial_pose_mode
        guess = self.mission['initial_guess']
        if mode == 'topic':
            self.log.info(f'publishing ROUGH initial pose guess {tuple(round(v, 2) for v in guess)} '
                          f'(true start {self.mission["start_pose"]})')
        elif mode == 'global':
            self.log.info('global localization: scattering particles over the whole map')
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            if mode == 'topic':
                self.node.initialpose_pub.publish(make_initial_pose(*guess, stamp=Time().to_msg()))
            elif mode == 'global' and self.node.global_loc.service_is_ready():
                call_service(self.node.global_loc, Empty.Request(), timeout_sec=5.0)
                mode = 'done'
            time.sleep(1.0)
            if self.node.estimate() is not None:
                self.node.event('initial_pose', mode=self.node.initial_pose_mode, guess=guess)
                return True
        raise RuntimeError('localization never published map -> base_footprint (is it running?)')

    def localize_by_spinning(self):
        """AMCL only updates when the robot MOVES (update_min_d / update_min_a). Spinning in place
        is the safest motion: it feeds AMCL new views without risking a collision."""
        if not self.node.actions.wait_for_servers(timeout=30, need_spin=True):
            return
        turns = 2 if self.node.initial_pose_mode == 'global' else 1
        for _ in range(turns):
            self.node.actions.spin_in_place(6.28, 40)
        cov = self.node.amcl_cov
        if cov:
            self.log.info(f'after spin: AMCL std x={math.sqrt(cov[0]):.2f} m, y={math.sqrt(cov[1]):.2f} m, '
                          f'yaw={math.sqrt(cov[2]):.2f} rad')

    # ----------------------------------------------------------- execution
    def run(self):
        n = self.node
        time.sleep(n.start_delay)
        self.set_initial_pose()
        if not self.wait_for_nav2():
            raise RuntimeError('Nav2 did not become active (is navigation launched?)')
        if n.pre_mission_spin:
            self.localize_by_spinning()
        goals = self.mission['goals']
        n.markers(goals, self.status)
        n.event('mission_started', name=self.mission['name'], method=n.method, goals=len(goals))
        t_start = n.now_s()

        if n.method in ('navigate_to_pose', 'raw_action'):
            for g in goals:
                self.run_goal_with_retries(g)
        else:
            self.run_batch(goals)

        if self.mission['retry_failed_at_end']:
            failed = [g for g in goals if self.status.get(g['index']) != 'SUCCEEDED']
            for g in failed:
                self.log.info(f'retrying failed goal {g["name"]} at the end of the mission')
                self.run_goal(g, attempt=99)

        total = n.now_s() - t_start
        ok = sum(1 for g in goals if self.status.get(g['index']) == 'SUCCEEDED')
        summary = {'mission': self.mission['name'], 'method': n.method, 'goals': len(goals),
                   'succeeded': ok, 'failed': len(goals) - ok, 'mission_time_s': round(total, 1),
                   'all_goals_complete': ok == len(goals)}
        self.write_results(summary)
        n.event('mission_finished', **summary)
        return summary

    def run_goal_with_retries(self, g):
        for attempt in range(1, self.mission['retries_per_goal'] + 2):
            if self.run_goal(g, attempt) == 'SUCCEEDED':
                return
            self.log.warn(f'goal {g["name"]} attempt {attempt} failed')

    def run_goal(self, g, attempt=1):
        n = self.node
        self.status[g['index']] = 'active'
        n.markers(self.mission['goals'], self.status)
        n.event('goal_started', index=g['index'], name=g['name'], attempt=attempt, x=g['x'], y=g['y'])
        t0, d0 = n.now_s(), n.truth_distance
        recoveries = 0
        pose = make_pose_stamped(g['x'], g['y'], g['yaw'], 'map', n.get_clock().now().to_msg())

        if n.method == 'raw_action':
            h = n.actions.navigate(g['x'], g['y'], g['yaw'], behavior_tree=n.behavior_tree)
            while not h.wait(0.2):
                if h.feedback is not None:
                    recoveries = max(recoveries, h.feedback.number_of_recoveries)
                if n.now_s() - t0 > g['timeout']:
                    self.log.warn(f'goal {g["name"]} timed out after {g["timeout"]} s -> cancel')
                    h.cancel()
                    break
            result = 'SUCCEEDED' if h.succeeded else ('TIMEOUT' if n.now_s() - t0 > g['timeout'] else h.status_name)
        else:
            from nav2_simple_commander.robot_navigator import TaskResult
            # BasicNavigator keeps the LAST feedback message of the previous goal until the
            # first one of the new goal arrives -> clear it, or we would count old recoveries.
            self.nav.feedback = None
            self.nav.goToPose(pose, behavior_tree=n.behavior_tree)
            timed_out = False
            while not self.nav.isTaskComplete():
                fb = self.nav.getFeedback()
                if fb is not None:
                    recoveries = max(recoveries, fb.number_of_recoveries)
                if n.now_s() - t0 > g['timeout']:
                    self.log.warn(f'goal {g["name"]} timed out after {g["timeout"]} s -> cancel')
                    self.nav.cancelTask()
                    timed_out = True
                    break
            r = self.nav.getResult()
            result = 'TIMEOUT' if timed_out else {TaskResult.SUCCEEDED: 'SUCCEEDED', TaskResult.FAILED: 'FAILED',
                                                  TaskResult.CANCELED: 'CANCELED'}.get(r, 'UNKNOWN')
        self.record(g, attempt, result, t0, d0, recoveries)
        return result

    def run_batch(self, goals):
        """follow_waypoints / through_poses: the whole list is ONE Nav2 task.
        We only learn the final outcome at the end, so per-goal rows are written afterwards."""
        from nav2_simple_commander.robot_navigator import TaskResult
        n = self.node
        poses = [make_pose_stamped(g['x'], g['y'], g['yaw'], 'map', n.get_clock().now().to_msg()) for g in goals]
        timeout = sum(g['timeout'] for g in goals)
        t_task = n.now_s()
        if n.method == 'follow_waypoints':
            self.nav.followWaypoints(poses)
        else:
            self.nav.goThroughPoses(poses, behavior_tree=n.behavior_tree)
        visits = {}                 # index -> [t_start, dist_start, t_end, dist_end]
        current, recoveries = None, 0
        while not self.nav.isTaskComplete():
            fb = self.nav.getFeedback()
            if fb is None:
                continue
            if n.method == 'follow_waypoints':
                idx = int(fb.current_waypoint)
            else:
                recoveries = max(recoveries, fb.number_of_recoveries)
                idx = len(goals) - int(fb.number_of_poses_remaining)
            idx = min(max(idx, 0), len(goals) - 1)
            if idx != current:
                if current is not None:
                    visits[current][2:] = [n.now_s(), n.truth_distance]
                visits[idx] = [n.now_s(), n.truth_distance, None, None]
                current = idx
                self.status[idx] = 'active'
                n.markers(goals, self.status)
                n.event('goal_started', index=idx, name=goals[idx]['name'], attempt=1,
                        x=goals[idx]['x'], y=goals[idx]['y'])
            if n.now_s() - t_task > timeout:
                self.log.warn('batch task timed out -> cancel')
                self.nav.cancelTask()
                break
        if current is not None:
            visits[current][2:] = [n.now_s(), n.truth_distance]
        ok = self.nav.getResult() == TaskResult.SUCCEEDED
        missed = set()
        if n.method == 'follow_waypoints' and self.nav.result_future and self.nav.result_future.result():
            res = self.nav.result_future.result().result
            missed = {int(m.index) for m in getattr(res, 'missed_waypoints', [])}
        for g in goals:
            i = g['index']
            if n.method == 'follow_waypoints':
                success = i in visits and i not in missed and (ok or i < (current or 0))
            else:   # through_poses: a pose counts only if we got past it, or the whole task succeeded
                success = ok or (current is not None and i < current)
            t0, d0 = (visits[i][0], visits[i][1]) if i in visits else (n.now_s(), n.truth_distance)
            self.record(g, 1, 'SUCCEEDED' if success else 'FAILED', t0, d0, recoveries,
                        t_end=visits[i][2] if i in visits else None,
                        d_end=visits[i][3] if i in visits else None)

    # ----------------------------------------------------------- bookkeeping
    def record(self, g, attempt, result, t0, d0, recoveries, t_end=None, d_end=None):
        n = self.node
        t_end = n.now_s() if t_end is None else t_end
        d_end = n.truth_distance if d_end is None else d_end
        duration = t_end - t0
        est = n.estimate()
        truth = n.truth
        row = {'index': g['index'], 'name': g['name'], 'attempt': attempt, 'method': n.method,
               'result': result, 'duration_s': round(duration, 2), 'recoveries': recoveries,
               'distance_m': round(d_end - d0, 2) if truth else None}
        if truth:   # how close did we REALLY get, and how wrong was our belief?
            row['final_position_error_m'] = round(math.hypot(truth[0] - g['x'], truth[1] - g['y']), 3)
            row['final_heading_error_rad'] = round(abs(normalize_angle(truth[2] - g['yaw'])), 3)
            if est:
                row['localization_error_m'] = round(math.hypot(truth[0] - est[0], truth[1] - est[1]), 3)
        self.rows.append(row)
        self.status[g['index']] = 'SUCCEEDED' if result == 'SUCCEEDED' else 'FAILED'
        n.markers(self.mission['goals'], self.status)
        n.event('goal_finished', **row)

    def write_results(self, summary):
        d = self.node.run_dir
        keys = sorted({k for r in self.rows for k in r}, key=lambda k: list(self.rows[0]).index(k)
                      if self.rows and k in self.rows[0] else 99)
        with open(os.path.join(d, 'mission_results.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(self.rows)
        write_json(os.path.join(d, 'mission_results.json'), {'summary': summary, 'goals': self.rows})
        self.log.info(f'results written to {d}/mission_results.csv')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    executor = spin_in_background(node)
    runner = None
    exit_code = 0
    try:
        runner = MissionRunner(node)
        summary = runner.run()
        node.get_logger().info(f'MISSION SUMMARY: {summary}')
        exit_code = 0 if summary['all_goals_complete'] else 2
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'mission failed: {e!r}')
        write_json(os.path.join(node.run_dir, 'mission_results.json'), {'error': repr(e)})
        exit_code = 1
    finally:
        ignore_sigint()
        if runner is not None and runner.nav is not None:
            runner.nav.destroy_node()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    main()
