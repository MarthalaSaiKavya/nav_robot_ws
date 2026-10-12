"""
lite_sim - a ~300-line 2D robot simulator written in Python.

WHY DOES THIS EXIST?
  Gazebo is the "real" simulator in this project, but it needs a decent GPU (the
  LiDAR is rendered) and several GB of RAM. lite_sim lets you run EVERY phase on a
  laptop or VM, and - more importantly - shows you that a simulator is not magic:
  it is a loop that (1) moves the robot according to /cmd_vel, (2) ray-casts the
  LiDAR, (3) publishes the same messages a real driver would.

  It is a drop-in replacement: same topics, same frames, same services as
  Gazebo + ros_gz_bridge. Swap with:   sim:=lite

WHAT IT SIMULATES
  * differential-drive kinematics with acceleration limits
  * collisions (the robot is a circle; it stops at walls; the wheels keep "spinning",
    so wheel odometry drifts while you push against a wall - just like real life)
  * a 360-degree LiDAR with Gaussian noise (ray-cast against the SDF room)
  * perfect wheel odometry (/wheel/odom_raw) - errors are added later by odometry_processor
  * an IMU (gyro z + noise), wheel joint states, ground truth, /clock
  * services /world/<name>/create | remove | set_pose  (spawn obstacles, teleport)

WHAT IT DOES NOT SIMULATE: inertia/dynamics, friction, 3D, sensor occlusion by height.

PUBLISHED   /clock /scan /wheel/odom_raw /ground_truth/odom /joint_states /imu
            /lite_sim/markers (obstacles for RViz, because there is no Gazebo window)
SUBSCRIBED  /cmd_vel (geometry_msgs/Twist)
"""
import math
import random

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from ..common.geometry import compose, normalize_angle, yaw_to_quaternion
from ..common.world_model import Box, World2D, shapes_from_model_sdf
from ..common.ros_utils import run_node


def _stamp(t_ns):
    """Integer nanoseconds -> builtin_interfaces/Time. (Using integers avoids float rounding
    that can make time jump BACKWARDS by a second - TF and robot_state_publisher hate that.)"""
    msg = TimeMsg()
    msg.sec = int(t_ns // 1_000_000_000)
    msg.nanosec = int(t_ns % 1_000_000_000)
    return msg


class LiteSim(Node):
    def __init__(self):
        super().__init__('lite_sim')
        p = self.declare_parameter
        self.world_file = p('world_file', '').value
        self.robot_name = p('robot_name', 'my_bot').value
        # robot geometry - keep in sync with my_bot_description/urdf/inc/properties.xacro
        self.robot_radius = p('robot_radius', 0.23).value
        self.wheel_sep = p('wheel_separation', 0.35).value
        self.wheel_radius = p('wheel_radius', 0.05).value
        self.max_v = p('max_linear_velocity', 0.6).value
        self.max_w = p('max_angular_velocity', 2.5).value
        self.max_acc_v = p('max_linear_acceleration', 1.5).value
        self.max_acc_w = p('max_angular_acceleration', 4.0).value
        self.cmd_timeout = p('cmd_vel_timeout', 0.5).value
        # LiDAR
        self.lidar_x = p('lidar_offset_x', 0.05).value
        self.lidar_height = p('lidar_height', 0.155).value
        self.n_rays = p('lidar_samples', 360).value
        self.range_min = p('lidar_range_min', 0.12).value
        self.range_max = p('lidar_range_max', 12.0).value
        self.lidar_noise = p('lidar_noise_std', 0.01).value
        # rates
        self.physics_rate = p('physics_rate', 100.0).value
        self.scan_rate = p('scan_rate', 10.0).value
        self.odom_rate = p('odom_rate', 50.0).value
        self.real_time_factor = p('real_time_factor', 1.0).value
        self.gyro_noise = p('gyro_noise_std', 0.002).value
        # start pose
        self.pose = [p('spawn_x', 0.0).value, p('spawn_y', 0.0).value, p('spawn_yaw', 0.0).value]
        seed = p('random_seed', 0).value
        self.rng = np.random.default_rng(seed if seed else None)
        random.seed(seed if seed else None)

        if not self.world_file:
            raise RuntimeError('lite_sim needs the parameter world_file (path to an SDF world)')
        self.world = World2D.from_sdf_file(self.world_file)
        world_name = p('world_name', '').value or self.world.world_name
        self.get_logger().info(self.world.summary())

        # simulation state
        self.sim_time = 0.0                # seconds (float, for maths)
        self.sim_ns = 0                    # nanoseconds (int, for time stamps)
        self.v = 0.0                       # actual velocities
        self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_time = -1e9
        self.odom = [0.0, 0.0, 0.0]        # perfect wheel odometry, starts at 0
        self.wheel_angles = [0.0, 0.0]
        self.in_collision = False
        self.angles = -math.pi + np.arange(self.n_rays) * (2 * math.pi / self.n_rays)
        self._tick = 0

        # publishers / subscribers
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom_raw', 10)
        self.gt_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', sensor_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/lite_sim/markers', 1)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)

        # Same services as Gazebo's UserCommands (bridged): spawn/remove/teleport
        from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
        base = f'/world/{world_name}'
        self.create_service(SpawnEntity, f'{base}/create', self._srv_create)
        self.create_service(DeleteEntity, f'{base}/remove', self._srv_remove)
        self.create_service(SetEntityPose, f'{base}/set_pose', self._srv_set_pose)

        # The main loop. NOTE: this node runs on WALL time (use_sim_time=false):
        # it is the one that CREATES simulated time and publishes it on /clock.
        self.dt = 1.0 / self.physics_rate
        self.dt_ns = int(round(1e9 / self.physics_rate))
        self.create_timer(self.dt / max(self.real_time_factor, 1e-3), self._step)
        self.create_timer(1.0, self._publish_markers)
        self.get_logger().info(
            f'lite_sim running: world "{world_name}", robot at ({self.pose[0]:.2f}, {self.pose[1]:.2f}), '
            f'services under {base}/')

    # ------------------------------------------------------------------ callbacks
    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = self.sim_time

    def _step(self):
        dt = self.dt
        self.sim_ns += self.dt_ns
        self.sim_time = self.sim_ns * 1e-9
        self._tick += 1

        # 1) Command -> target velocity (stop if commands stop arriving: a safety feature
        #    every real motor driver has, so a crashed Nav2 does not drive into a wall).
        tv, tw = self.cmd if (self.sim_time - self.cmd_time) < self.cmd_timeout else (0.0, 0.0)
        tv = max(-self.max_v, min(self.max_v, tv))
        tw = max(-self.max_w, min(self.max_w, tw))
        # 2) Acceleration limits (motors cannot change speed instantly)
        self.v += max(-self.max_acc_v * dt, min(self.max_acc_v * dt, tv - self.v))
        self.w += max(-self.max_acc_w * dt, min(self.max_acc_w * dt, tw - self.w))

        # 3) Integrate the true pose (unicycle model), refuse moves that collide
        x, y, th = self.pose
        nth = normalize_angle(th + self.w * dt)
        nx = x + self.v * math.cos(th + 0.5 * self.w * dt) * dt
        ny = y + self.v * math.sin(th + 0.5 * self.w * dt) * dt
        if self.world.distance_to_obstacles(nx, ny, self.lidar_height) < self.robot_radius:
            if not self.in_collision:
                self.get_logger().warn('COLLISION: robot touched an obstacle (wheels slip, odometry drifts)')
            self.in_collision = True
            self.pose = [x, y, nth]          # rotation is still allowed
        else:
            self.in_collision = False
            self.pose = [nx, ny, nth]

        # 4) Wheel odometry integrates what the WHEELS did (even when blocked = slip)
        ox, oy, oth = self.odom
        self.odom = [ox + self.v * math.cos(oth + 0.5 * self.w * dt) * dt,
                     oy + self.v * math.sin(oth + 0.5 * self.w * dt) * dt,
                     normalize_angle(oth + self.w * dt)]
        self.wheel_angles[0] += (self.v - self.w * self.wheel_sep / 2) / self.wheel_radius * dt
        self.wheel_angles[1] += (self.v + self.w * self.wheel_sep / 2) / self.wheel_radius * dt

        stamp = _stamp(self.sim_ns)
        self.clock_pub.publish(Clock(clock=stamp))

        if self._tick % max(1, int(round(self.physics_rate / self.odom_rate))) == 0:
            self._publish_odom(stamp)
        if self._tick % max(1, int(round(self.physics_rate / self.scan_rate))) == 0:
            self._publish_scan(stamp)

    def _publish_odom(self, stamp):
        def odom_msg(frame, pose):
            m = Odometry()
            m.header.stamp = stamp
            m.header.frame_id = frame
            m.child_frame_id = 'base_footprint'
            m.pose.pose.position.x, m.pose.pose.position.y = pose[0], pose[1]
            qx, qy, qz, qw = yaw_to_quaternion(pose[2])
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = qz, qw
            m.twist.twist.linear.x = self.v
            m.twist.twist.angular.z = self.w
            return m
        self.odom_pub.publish(odom_msg('odom', self.odom))
        self.gt_pub.publish(odom_msg('world', self.pose))

        js = JointState()
        js.header.stamp = stamp
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = list(self.wheel_angles)
        js.velocity = [(self.v - self.w * self.wheel_sep / 2) / self.wheel_radius,
                       (self.v + self.w * self.wheel_sep / 2) / self.wheel_radius]
        self.js_pub.publish(js)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        qx, qy, qz, qw = yaw_to_quaternion(self.pose[2])
        imu.orientation.z, imu.orientation.w = qz, qw
        imu.angular_velocity.z = self.w + float(self.rng.normal(0.0, self.gyro_noise))
        imu.linear_acceleration.z = 9.81
        imu.orientation_covariance[0] = imu.orientation_covariance[4] = imu.orientation_covariance[8] = 1e-3
        imu.angular_velocity_covariance[8] = self.gyro_noise ** 2 + 1e-9
        imu.linear_acceleration_covariance[0] = 1e-2
        self.imu_pub.publish(imu)

    def _publish_scan(self, stamp):
        # LiDAR pose in the world = robot pose (+) mounting offset
        lx, ly, lth = compose(self.pose, (self.lidar_x, 0.0, 0.0))
        ranges = self.world.raycast(lx, ly, self.angles + lth, self.range_max, self.lidar_height)
        hit = np.isfinite(ranges)
        ranges[hit] += self.rng.normal(0.0, self.lidar_noise, hit.sum())
        ranges[hit & (ranges < self.range_min)] = np.inf      # too close -> no return
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = float(self.angles[0])
        msg.angle_max = float(self.angles[-1])
        msg.angle_increment = float(2 * math.pi / self.n_rays)
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.scan_rate
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = ranges.astype(np.float32).tolist()
        self.scan_pub.publish(msg)

    # ------------------------------------------------------------------ services
    def _srv_create(self, req, res):
        ef = req.entity_factory
        try:
            sdf = ef.sdf
            if not sdf and ef.sdf_filename:
                with open(ef.sdf_filename) as f:
                    sdf = f.read()
            q = ef.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            name, shapes = shapes_from_model_sdf(sdf, ef.pose.position.x, ef.pose.position.y,
                                                 ef.pose.position.z, yaw, ef.name or None)
            if name in self.world.model_names():
                self.get_logger().warn(f'create: "{name}" already exists')
                res.success = False
                return res
            self.world.add_shapes(shapes)
            self.get_logger().info(f'spawned "{name}" at ({ef.pose.position.x:.2f}, {ef.pose.position.y:.2f})')
            res.success = True
        except Exception as e:
            self.get_logger().error(f'create failed: {e}')
            res.success = False
        self._publish_markers()
        return res

    def _srv_remove(self, req, res):
        n = self.world.remove_model(req.entity.name)
        res.success = n > 0
        self.get_logger().info(f'remove "{req.entity.name}" -> {"OK" if n else "not found"}')
        self._publish_markers(clear_first=True)
        return res

    def _srv_set_pose(self, req, res):
        q = req.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = req.pose.position.x, req.pose.position.y
        if req.entity.name == self.robot_name:
            # Teleport ("kidnap") the robot. Wheel odometry is NOT reset - exactly like
            # picking up a real robot and putting it somewhere else.
            self.pose = [x, y, yaw]
            self.v = self.w = 0.0
            res.success = True
            self.get_logger().info(f'teleported robot to ({x:.2f}, {y:.2f}, {yaw:.2f})')
        else:
            res.success = self.world.move_model(req.entity.name, x, y, yaw)
        self._publish_markers(clear_first=True)
        return res

    # ------------------------------------------------------------------ visualisation
    def _publish_markers(self, clear_first=False):
        arr = MarkerArray()
        if clear_first:
            m = Marker()
            m.action = Marker.DELETEALL
            arr.markers.append(m)
        for i, s in enumerate(self.world.shapes):
            m = Marker()
            m.header.frame_id = 'map'          # map == world in this project (see world SDF)
            m.ns = 'lite_sim_world'
            m.id = i
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y = s.cx, s.cy
            m.pose.position.z = s.z_min + s.height / 2
            if isinstance(s, Box):
                m.type = Marker.CUBE
                m.scale.x, m.scale.y, m.scale.z = s.sx, s.sy, s.height
                _, _, qz, qw = yaw_to_quaternion(s.yaw)
                m.pose.orientation.z, m.pose.orientation.w = qz, qw
            else:
                m.type = Marker.CYLINDER
                m.scale.x = m.scale.y = 2 * s.radius
                m.scale.z = s.height
                m.pose.orientation.w = 1.0
            spawned = s.name.split('/')[0].startswith('scenario_')
            m.color.r, m.color.g, m.color.b, m.color.a = ((0.95, 0.45, 0.1, 0.8) if spawned
                                                          else (0.6, 0.6, 0.65, 0.35))
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    run_node(LiteSim, args)


if __name__ == '__main__':
    main()
