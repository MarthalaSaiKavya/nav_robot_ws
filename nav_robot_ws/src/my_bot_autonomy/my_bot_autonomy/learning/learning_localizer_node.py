"""
learning_localizer - our own particle filter (learning/particle_filter.py) as an AMCL replacement.

    ros2 launch my_bot_bringup phase4_localization.launch.py localization_method:=learning_pf

Inputs : /map (from map_server), /scan, TF odom -> base_footprint, /initialpose (RViz "2D Pose Estimate")
Outputs: /learning/particles (PoseArray, RViz), /learning/pose (PoseWithCovarianceStamped)
         TF map -> odom   (when publish_tf:=true, i.e. when it REPLACES AMCL)

How map -> odom is computed (the same trick AMCL and every SLAM use):
   we estimate  map -> base_footprint  (where the robot is in the map)
   odometry gives odom -> base_footprint
   so           map -> odom = (map -> base) o (odom -> base)^-1
Publishing map->odom (instead of map->base) keeps the TF tree a TREE: base_footprint
keeps ONE parent (odom), and the smooth odometry is corrected by a slowly changing offset.
"""
import math
import threading

import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from ..common.geometry import compose, inverse, yaw_to_quaternion
from ..common.map_io import grid_from_msg
from ..common.ros_utils import lookup_pose_2d, quaternion_msg, run_node, yaw_of
from .particle_filter import ParticleFilter


class LearningLocalizer(Node):
    def __init__(self):
        super().__init__('learning_localizer')
        p = self.declare_parameter
        self.n = p('num_particles', 500).value
        self.publish_tf = p('publish_tf', True).value
        self.update_min_d = p('update_min_d', 0.15).value
        self.update_min_a = p('update_min_a', 0.15).value
        self.lidar_offset = (p('lidar_offset_x', 0.05).value, 0.0)
        self.init = (p('initial_pose.x', 0.0).value, p('initial_pose.y', 0.0).value, p('initial_pose.yaw', 0.0).value)
        self.set_initial_pose = p('set_initial_pose', False).value
        self.global_init = p('global_localization', False).value
        self.transform_tolerance = p('transform_tolerance', 0.5).value
        self.pf = None
        self.lock = threading.Lock()
        self.last_odom = None
        self.map_to_odom = (0.0, 0.0, 0.0)
        self.have_estimate = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._on_initialpose, 10)
        self.particles_pub = self.create_publisher(PoseArray, '/learning/particles', 1)
        pose_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/learning/pose', pose_qos)
        if self.publish_tf:
            self.create_timer(0.05, self._broadcast)

    def _on_map(self, msg):
        grid, res, origin = grid_from_msg(msg)
        with self.lock:
            self.pf = ParticleFilter(grid, res, origin, self.n)
            if self.global_init:
                self.pf.init_uniform()
            else:
                self.pf.init_gaussian(*self.init)
            self.have_estimate = self.set_initial_pose or self.global_init
        self.get_logger().info(f'map received ({grid.shape[1]}x{grid.shape[0]}); particles initialised '
                               f'{"uniformly" if self.global_init else "around " + str(self.init)}')

    def _on_initialpose(self, msg):
        p = msg.pose.pose
        std_xy = math.sqrt(max(msg.pose.covariance[0], 0.01))
        std_yaw = math.sqrt(max(msg.pose.covariance[35], 0.01))
        with self.lock:
            if self.pf is not None:
                self.pf.init_gaussian(p.position.x, p.position.y, yaw_of(p.orientation), std_xy, std_yaw)
                self.have_estimate = True
                self.last_odom = None
        self.get_logger().info(f'initial pose received ({p.position.x:.2f}, {p.position.y:.2f})')

    def _on_scan(self, scan):
        stamp = Time.from_msg(scan.header.stamp)
        odom = lookup_pose_2d(self.tf_buffer, 'odom', 'base_footprint', stamp, 0.1)
        if odom is None or self.pf is None or not self.have_estimate:
            return
        with self.lock:
            if self.last_odom is None:
                self.last_odom = odom
                moved = True
            else:
                d = math.hypot(odom[0] - self.last_odom[0], odom[1] - self.last_odom[1])
                a = abs(math.atan2(math.sin(odom[2] - self.last_odom[2]), math.cos(odom[2] - self.last_odom[2])))
                moved = d > self.update_min_d or a > self.update_min_a
                if moved:                          # like AMCL: only filter after real motion
                    self.pf.predict(self.last_odom, odom)
                    self.last_odom = odom
            if not moved:
                return
            angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
            self.pf.update(np.asarray(scan.ranges, dtype=float), angles, self.lidar_offset)
            self.pf.resample()
            est, var = self.pf.estimate()
            self.map_to_odom = compose(est, inverse(odom))
            particles = self.pf.particles.copy()
        self._publish(est, var, particles, scan.header.stamp)

    def _publish(self, est, var, particles, stamp):
        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = stamp
        for x, y, th in particles[::max(1, len(particles) // 300)]:
            ps = Pose()
            ps.position.x, ps.position.y = float(x), float(y)
            ps.orientation = quaternion_msg(float(th))
            pa.poses.append(ps)
        self.particles_pub.publish(pa)
        msg = PoseWithCovarianceStamped()
        msg.header = pa.header
        msg.pose.pose.position.x, msg.pose.pose.position.y = est[0], est[1]
        msg.pose.pose.orientation = quaternion_msg(est[2])
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = var
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)

    def _broadcast(self):
        if not self.have_estimate:
            return
        m2o = self.map_to_odom
        t = TransformStamped()
        # future-dated by transform_tolerance, exactly like AMCL, so consumers never wait
        t.header.stamp = (self.get_clock().now() + Duration(seconds=self.transform_tolerance)).to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x, t.transform.translation.y = m2o[0], m2o[1]
        _, _, qz, qw = yaw_to_quaternion(m2o[2])
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    run_node(LearningLocalizer, args, multithreaded=True)


if __name__ == '__main__':
    main()
