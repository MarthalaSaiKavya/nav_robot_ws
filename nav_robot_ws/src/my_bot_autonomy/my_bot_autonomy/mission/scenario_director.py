"""
scenario_director - changes the world DURING a mission to test Nav2 (Phases 5-7).

It listens to /mission/events (published by mission_runner) and to the robot pose,
and when a TRIGGER fires it performs an ACTION through the simulator services
(/world/nav_room/create|remove - works with Gazebo AND with sim:=lite).

SCENARIO FILE (my_bot_bringup/config/scenarios/*.yaml):

    scenario:
      name: demo
      events:
        - name: surprise_box
          trigger: {type: goal_started, goal: 1, delay: 4.0}
          action:  {type: spawn_on_path, distance_ahead: 1.6, size: [0.5, 0.5, 0.6]}
        - name: door_block
          trigger: {type: robot_near, x: 3.0, y: 0.8, radius: 1.5, during_goal: 2}
          action:  {type: spawn_box, x: 3.0, y: 1.5, yaw: 0.0, size: [1.3, 0.35, 0.8]}
        - name: cage
          trigger: {type: goal_started, goal: 3}
          action:  {type: spawn_cage, inner_radius: 0.75}
        - name: open_cage
          trigger: {type: goal_finished, goal: 3}
          action:  {type: remove, names: [cage]}

TRIGGERS  goal_started | goal_finished  (+ optional result: FAILED, delay: s)
          robot_near {x, y, radius}     (+ optional during_goal: i)
          time {after: s}               (seconds after mission_started)
ACTIONS   spawn_box      a box at a fixed place
          spawn_on_path  a box ON Nav2's current global plan, distance_ahead metres in front
                         of the robot (= "an obstacle appears that the map does not know about")
          spawn_cage     4 boxes around the robot -> the path is FULLY occluded (hard block)
          remove         remove the obstacles spawned by the listed events
Every event fires once (unless once: false). Spawned models are named scenario_<event>[_k].
"""
import json
import math
import threading
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from ..common.ros_utils import ignore_sigint, lookup_pose_2d, spin_in_background
from ..common.sim_services import SimWorldClient
from .mission_io import load_scenario


class ScenarioDirector(Node):
    def __init__(self):
        super().__init__('scenario_director')
        p = self.declare_parameter
        self.scenario = load_scenario(p('scenario_file', 'demo_scenario.yaml').value)
        self.world_name = p('world_name', 'nav_room').value
        self.backend = p('backend', 'ros').value                     # ros | gz_cli
        self.robot_frame = p('robot_frame', 'base_footprint').value
        self.sim = SimWorldClient(self, self.world_name, self.backend)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.plan = None
        self.current_goal = None
        self.mission_t0 = None
        self.spawned = {}          # event name -> [model names]
        self.queue = []            # (due_time, event, context) waiting to be executed
        self.lock = threading.Lock()
        self.create_subscription(String, '/mission/events', self._on_event, 10)
        self.create_subscription(Path, '/plan', self._on_plan, 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/scenario/markers', 1)
        self.get_logger().info(f'scenario "{self.scenario["name"]}" with {len(self.scenario["events"])} events')
        self.worker = threading.Thread(target=self._loop, daemon=True)
        self.worker.start()

    # ------------------------------------------------------------- inputs
    def _on_plan(self, msg):
        self.plan = msg

    def _on_event(self, msg):
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        kind = ev.get('event')
        if kind == 'mission_started':
            self.mission_t0 = time.time()
        if kind == 'goal_started':
            self.current_goal = ev.get('index')
        for e in self.scenario['events']:
            trig = e['trigger']
            if trig.get('type') != kind or (e['once'] and e['fired']):
                continue
            if 'goal' in trig and int(trig['goal']) != ev.get('index'):
                continue
            if 'result' in trig and trig['result'] != ev.get('result'):
                continue
            self._schedule(e, float(trig.get('delay', 0.0)))

    def _schedule(self, e, delay):
        e['fired'] += 1
        with self.lock:
            self.queue.append((time.time() + delay, e))
        self.get_logger().info(f'trigger "{e["name"]}" fired (action in {delay:.1f} s)')

    # ------------------------------------------------------------- main loop
    def _loop(self):
        while rclpy.ok():
            robot = lookup_pose_2d(self.tf_buffer, 'map', self.robot_frame)
            for e in self.scenario['events']:
                trig = e['trigger']
                if e['once'] and e['fired']:
                    continue
                if trig.get('type') == 'robot_near' and robot is not None:
                    if 'during_goal' in trig and int(trig['during_goal']) != self.current_goal:
                        continue
                    if math.hypot(robot[0] - trig['x'], robot[1] - trig['y']) <= trig.get('radius', 1.0):
                        self._schedule(e, float(trig.get('delay', 0.0)))
                elif trig.get('type') == 'time' and self.mission_t0 is not None:
                    if time.time() - self.mission_t0 >= float(trig.get('after', 0.0)):
                        self._schedule(e, 0.0)
            with self.lock:
                due = [q for q in self.queue if q[0] <= time.time()]
                self.queue = [q for q in self.queue if q[0] > time.time()]
            for _, e in due:
                self._execute(e, robot)
            time.sleep(0.2)

    def _execute(self, e, robot):
        a = e['action']
        kind = a.get('type')
        name = 'scenario_' + e['name']
        size = a.get('size', [0.5, 0.5, 0.6])
        log = self.get_logger()
        if kind == 'spawn_box':
            self._spawn(e, name, a['x'], a['y'], a.get('yaw', 0.0), size)
        elif kind == 'spawn_on_path':
            spot = self._point_on_plan(robot, float(a.get('distance_ahead', 1.5)))
            if spot is None and 'fallback' in a:
                spot = (a['fallback']['x'], a['fallback']['y'], 0.0)
                log.warn('no usable /plan, using fallback position')
            if spot is not None:
                self._spawn(e, name, spot[0], spot[1], spot[2], size)
        elif kind == 'spawn_cage':
            if robot is None:
                log.error('spawn_cage: robot pose unknown')
                return
            r = float(a.get('inner_radius', 0.75))
            t = float(a.get('thickness', 0.25))
            length = 2 * (r + t)
            for k, (dx, dy, yaw) in enumerate([(r + t / 2, 0, 0), (-r - t / 2, 0, 0),
                                                 (0, r + t / 2, math.pi / 2), (0, -r - t / 2, math.pi / 2)]):
                self._spawn(e, f'{name}_{k}', robot[0] + dx, robot[1] + dy, yaw, [t, length, size[2]])
        elif kind == 'remove':
            for ev_name in a.get('names', []):
                for model in self.spawned.pop(ev_name, []):
                    self.sim.remove(model)
        else:
            log.error(f'unknown action type {kind}')
        self._publish_markers()

    def _spawn(self, e, model, x, y, yaw, size):
        if self.sim.spawn_box(model, x, y, yaw, *size):
            self.spawned.setdefault(e['name'], []).append(model)
            self._markers_cache = getattr(self, '_markers_cache', {})
            self._markers_cache[model] = (x, y, yaw, size)

    def _point_on_plan(self, robot, ahead):
        """Walk along the latest global plan until `ahead` metres from the robot."""
        if self.plan is None or robot is None or len(self.plan.poses) < 2:
            return None
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in self.plan.poses]
        # start from the plan point closest to the robot
        i0 = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - robot[0], pts[i][1] - robot[1]))
        travelled = 0.0
        for i in range(i0, len(pts) - 1):
            seg = math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            if travelled + seg >= ahead:
                f = (ahead - travelled) / max(seg, 1e-6)
                x = pts[i][0] + f * (pts[i + 1][0] - pts[i][0])
                y = pts[i][1] + f * (pts[i + 1][1] - pts[i][1])
                yaw = math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0])
                return (x, y, yaw)
            travelled += seg
        return None   # path shorter than `ahead`: too close to the goal to block it fairly

    def _publish_markers(self):
        arr = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        arr.markers.append(m)
        live = {mdl for models in self.spawned.values() for mdl in models}
        for k, (model, (x, y, yaw, size)) in enumerate(getattr(self, '_markers_cache', {}).items()):
            if model not in live:
                continue
            c = Marker()
            c.header.frame_id = 'map'
            c.ns, c.id, c.type = 'scenario', k, Marker.CUBE
            c.pose.position.x, c.pose.position.y, c.pose.position.z = x, y, size[2] / 2
            c.pose.orientation.z, c.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            c.scale.x, c.scale.y, c.scale.z = size
            c.color.r, c.color.g, c.color.b, c.color.a = 1.0, 0.45, 0.0, 0.7
            arr.markers.append(c)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioDirector()
    executor = spin_in_background(node)
    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        ignore_sigint()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
