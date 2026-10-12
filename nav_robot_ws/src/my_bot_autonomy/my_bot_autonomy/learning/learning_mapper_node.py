"""
learning_mapper - occupancy mapping with KNOWN poses (learning/occupancy_mapping.py) as a ROS node.

    # see how a map looks when the poses are PERFECT (ground truth):
    ros2 run my_bot_autonomy learning_mapper --ros-args -p pose_source:=ground_truth
    # ...and when the poses come from drifting ODOMETRY only (no SLAM correction):
    ros2 run my_bot_autonomy learning_mapper --ros-args -p pose_source:=odom
    # ...or using the poses estimated by a running SLAM (map -> base_footprint):
    ros2 run my_bot_autonomy learning_mapper --ros-args -p pose_source:=tf_map

Output: /learning/map (nav_msgs/OccupancyGrid) - add it in RViz (it is in navigation.rviz).

As a SLAM replacement (slam_method:=learning_mapper):
  publish_as_slam:=true publishes on /map and broadcasts map -> odom computed from the
  ground truth. That is "mapping with a perfect localizer" - it lets the explorer and Nav2
  run on top of your own mapper, and shows the difference between MAPPING and SLAM.
"""
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from ..common.geometry import compose, inverse, yaw_to_quaternion
from ..common.map_io import msg_from_grid
from ..common.ros_utils import lookup_pose_2d, run_node, yaw_of
from .occupancy_mapping import LogOddsMap


class LearningMapper(Node):
    def __init__(self):
        super().__init__('learning_mapper')
        p = self.declare_parameter
        self.pose_source = p('pose_source', 'ground_truth').value    # ground_truth | odom | tf_map
        self.publish_as_slam = p('publish_as_slam', False).value
        self.lidar_offset = (p('lidar_offset_x', 0.05).value, 0.0, 0.0)
        size = p('map_size', 12.0).value
        res = p('resolution', 0.05).value
        self.max_range = p('max_range', 8.0).value
        # map -> odom is stamped this far into the future (the same trick as slam_toolbox's
        # transform_timeout and AMCL's transform_tolerance): consumers asking for "now" find it.
        self.transform_tolerance = p('transform_tolerance', 0.5).value
        self.map = LogOddsMap(size, size, res, (-size / 2, -size / 2))
        self.frame = 'odom' if self.pose_source == 'odom' else 'map'
        self.truth = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_as_slam else None
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 20)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map' if self.publish_as_slam else '/learning/map', qos)
        self.create_timer(1.0, self._publish_map)
        if self.publish_as_slam:
            self.create_timer(0.05, self._publish_map_to_odom)
        self.scans = 0
        self.get_logger().info(f'learning_mapper: pose_source={self.pose_source}, publish_as_slam={self.publish_as_slam}')

    def _on_truth(self, msg):
        p = msg.pose.pose
        self.truth = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _robot_pose(self):
        if self.pose_source == 'ground_truth':
            return self.truth
        if self.pose_source == 'odom':
            return lookup_pose_2d(self.tf_buffer, 'odom', 'base_footprint')
        return lookup_pose_2d(self.tf_buffer, 'map', 'base_footprint')

    def _on_scan(self, scan):
        pose = self._robot_pose()
        if pose is None:
            return
        self.scans += 1
        if self.scans % 2:          # every 2nd scan is plenty (5 Hz) and saves CPU
            return
        sensor = compose(pose, self.lidar_offset)
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        self.map.update_scan(sensor, np.asarray(scan.ranges, dtype=float), angles,
                             min(self.max_range, scan.range_max))

    def _publish_map(self):
        grid = self.map.to_occupancy()
        self.map_pub.publish(msg_from_grid(grid, self.map.res, self.map.origin, self.frame,
                                           self.get_clock().now().to_msg()))

    def _publish_map_to_odom(self):
        """map -> odom = (map -> base) o (odom -> base)^-1, with map -> base = ground truth."""
        odom = lookup_pose_2d(self.tf_buffer, 'odom', 'base_footprint')
        if odom is None or self.truth is None:
            return
        m2o = compose(self.truth, inverse(odom))
        t = TransformStamped()
        t.header.stamp = (self.get_clock().now() + Duration(seconds=self.transform_tolerance)).to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x, t.transform.translation.y = m2o[0], m2o[1]
        _, _, qz, qw = yaw_to_quaternion(m2o[2])
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    run_node(LearningMapper, args)


if __name__ == '__main__':
    main()
