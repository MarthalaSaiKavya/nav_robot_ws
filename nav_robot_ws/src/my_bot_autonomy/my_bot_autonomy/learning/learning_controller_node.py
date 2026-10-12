"""
learning_controller - follows /learning/plan with pure pursuit (learning/pure_pursuit.py).

Publishes /cmd_vel directly. Has ONE safety feature: if the LiDAR sees something closer
than stop_distance in front, it stops (a crude version of Nav2's collision_monitor).
It does NOT avoid obstacles or re-plan - that is exactly what Nav2's local costmap +
controller + behaviour tree add. Drive it into a spawned box and compare with Nav2!
"""
import math

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from ..common.ros_utils import lookup_pose_2d, run_node
from .pure_pursuit import PurePursuit


class LearningController(Node):
    def __init__(self):
        super().__init__('learning_controller')
        p = self.declare_parameter
        self.pp = PurePursuit(lookahead=p('lookahead', 0.6).value, max_v=p('max_linear_velocity', 0.3).value,
                              max_w=p('max_angular_velocity', 1.5).value,
                              goal_tolerance=p('goal_tolerance', 0.15).value)
        self.stop_distance = p('stop_distance', 0.35).value
        self.path = None
        self.front_clear = True
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Path, '/learning/plan', self._on_path, 1)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self._control)

    def _on_path(self, msg):
        self.path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.get_logger().info(f'new path with {len(self.path)} points')

    def _on_scan(self, msg):
        ang = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        r = np.asarray(msg.ranges, dtype=float)
        front = (np.abs(ang) < 0.45) & np.isfinite(r)
        self.front_clear = not np.any(r[front] < self.stop_distance)

    def _control(self):
        if not self.path:
            return
        pose = lookup_pose_2d(self.tf_buffer, 'map', 'base_footprint')
        if pose is None:
            return
        v, w, done = self.pp.compute(pose, self.path)
        if done:
            self.get_logger().info('goal reached')
            self.path = None
            v = w = 0.0
        elif v > 0 and not self.front_clear:
            v = 0.0       # obstacle ahead: stop (but allow turning)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)


def main(args=None):
    run_node(LearningController, args)


if __name__ == '__main__':
    main()
