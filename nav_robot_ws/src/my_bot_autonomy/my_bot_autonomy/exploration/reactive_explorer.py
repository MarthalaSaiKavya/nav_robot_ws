"""
reactive_explorer - exploration WITHOUT a map or a planner (a contrast to frontier exploration).

    ros2 launch my_bot_bringup phase8_exploration.launch.py explorer:=reactive reactive_mode:=wall_follow

"Reactive" = the robot only looks at the CURRENT laser scan and reacts; it never
plans. Nav2 is not used at all - this node publishes /cmd_vel directly.

  random_walk   drive straight; when something is closer than `safe_distance` ahead,
                turn on the spot in a random direction until the way is clear.
                (Roomba-style. Covers a room eventually, very inefficiently.)
  wall_follow   a 2-state machine:
                  FIND_WALL  drive straight until something is ahead, turn left so the
                             wall ends up on the RIGHT, then ->
                  FOLLOW     keep the wall on the right at `wall_distance` (P-controller).
                             Wall lost (outer corner)? curve right around it; still no
                             wall after `lost_wall_timeout` s -> FIND_WALL again.
                (The "maze solving" rule. Great at tracing the boundary, but it never
                visits free-standing obstacles in the middle of a room.)

Run either one with SLAM and compare the resulting map / time with frontier exploration
- that comparison is the lesson. Stops after `duration` seconds or when the map stops
growing for `stall_timeout` seconds, then saves the map like frontier_explorer.
"""
import math
import os
import random
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ..common.ros_utils import ignore_sigint, make_run_dir, write_json
from ..mapping.map_saving import save_map_with_fallbacks, serialize_pose_graph


class ReactiveExplorer(Node):
    def __init__(self):
        super().__init__('reactive_explorer')
        p = self.declare_parameter
        self.mode = p('mode', 'wall_follow').value                 # random_walk | wall_follow
        self.speed = p('linear_speed', 0.25).value
        self.turn_speed = p('angular_speed', 0.8).value
        self.safe_distance = p('safe_distance', 0.55).value
        self.wall_distance = p('wall_distance', 0.6).value
        self.kp = p('wall_kp', 1.5).value
        self.lost_wall_timeout = p('lost_wall_timeout', 15.0).value
        self.duration = p('duration', 300.0).value
        self.stall_timeout = p('stall_timeout', 60.0).value
        self.run_dir = make_run_dir(p('run_dir', '').value)
        self.map_name = p('map_name', 'explored_map').value
        self.exit_on_done = p('exit_on_done', True).value

        self.scan = None
        self.known_cells = 0
        self.last_growth = time.time()
        self.turning = 0                     # random walk: +1/-1 while turning away
        self.state = 'FIND_WALL'             # wall follow: FIND_WALL | FOLLOW
        self.wall_seen = time.time()         # wall follow: last time a wall was on the right
        self.t0 = time.time()
        self.done = False
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_timer(0.1, self._control)
        self.get_logger().info(f'reactive exploration: mode={self.mode} for up to {self.duration:.0f} s')

    def _on_scan(self, msg):
        self.scan = msg

    def _on_map(self, msg):
        known = int(np.count_nonzero(np.asarray(msg.data, dtype=np.int8) >= 0))
        if known > self.known_cells + 50:          # the map is still growing
            self.known_cells = known
            self.last_growth = time.time()

    def _sector_min(self, a0, a1):
        """Smallest range between angles a0..a1 (radians, robot frame, 0 = ahead)."""
        s = self.scan
        angles = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
        angles = (angles + math.pi) % (2 * math.pi) - math.pi
        r = np.asarray(s.ranges, dtype=float)
        sel = (angles >= a0) & (angles <= a1) & np.isfinite(r)
        return float(r[sel].min()) if sel.any() else float('inf')

    def _control(self):
        if self.done or self.scan is None:
            return
        elapsed = time.time() - self.t0
        if elapsed > self.duration or (elapsed > 30 and time.time() - self.last_growth > self.stall_timeout):
            self.done = True
            self.cmd_pub.publish(Twist())
            threading.Thread(target=self._finish, daemon=True).start()
            return
        cmd = Twist()
        front = self._sector_min(-0.5, 0.5)
        if self.mode == 'random_walk':
            if self.turning:
                cmd.angular.z = self.turning * self.turn_speed
                if front > self.safe_distance * 1.6:
                    self.turning = 0
            elif front < self.safe_distance:
                self.turning = random.choice((-1, 1))
            else:
                cmd.linear.x = self.speed
        else:  # wall_follow (wall on the right)
            right = self._sector_min(-1.9, -1.2)
            if right < 2.0 * self.wall_distance:
                self.wall_seen = time.time()
            if self.state == 'FIND_WALL':
                if front < self.safe_distance:
                    cmd.angular.z = self.turn_speed        # wall ahead: turn left (puts it on the right)
                    self._set_state('FOLLOW')
                else:
                    cmd.linear.x = self.speed              # open space: go straight until we hit a wall
            elif front < self.safe_distance:
                cmd.angular.z = self.turn_speed            # inner corner: turn left on the spot
            elif right > 2.0 * self.wall_distance:
                if time.time() - self.wall_seen > self.lost_wall_timeout:
                    self._set_state('FIND_WALL')           # circled too long: look for a new wall
                # outer corner: curve right with radius ~ wall_distance to stay around it
                cmd.linear.x = self.speed * 0.6
                cmd.angular.z = -cmd.linear.x / max(self.wall_distance, 0.3)
            else:
                err = self.wall_distance - right           # >0 too close -> turn left
                cmd.linear.x = self.speed
                cmd.angular.z = max(-1.0, min(1.0, self.kp * err))
        self.cmd_pub.publish(cmd)

    def _set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'[reactive] {self.state} -> {state}')
            self.state = state
            self.wall_seen = time.time()

    def _finish(self):
        base = os.path.join(self.run_dir, self.map_name)
        method = save_map_with_fallbacks(self, base)
        stats = {'strategy': f'reactive_{self.mode}', 'duration_s': round(time.time() - self.t0, 1),
                 'map_yaml': base + '.yaml' if method else None, 'map_save_method': method,
                 'pose_graph_saved': serialize_pose_graph(self, base), 'success': method is not None}
        write_json(os.path.join(self.run_dir, 'exploration_result.json'), stats)
        self.get_logger().info(f'reactive exploration done: {stats}')
        if self.exit_on_done:
            time.sleep(1.0)
            rclpy.try_shutdown()


def main(args=None):
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    rclpy.init(args=args)
    node = ReactiveExplorer()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ignore_sigint()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
