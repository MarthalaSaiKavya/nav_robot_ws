"""
odometry_processor - turns raw wheel odometry into the robot's /odom, with
OPTIONAL, CONFIGURABLE drift, and publishes the odom -> base_footprint TF.

WHY?
  The problem statement says "fuse noisy laser scans with DRIFTING odometry".
  Simulated wheels are almost perfect, so SLAM would look trivially easy. This node
  re-creates the classic error sources of real wheel encoders so you can SEE why
  SLAM / AMCL are needed:

    linear_scale_error   wheel radius 2% too small in software -> every metre reads 1.02 m
    angular_scale_error  wheel separation slightly wrong -> every 90 deg turn reads 87 deg
                         (the error that hurts maps the most!)
    *_noise_std          random jitter, grows like a random walk
    angular_bias         constant gyro-like drift (rad/s) while moving

  Profiles (my_bot_bringup/config/odometry/*.yaml):  perfect | realistic | terrible

HOW ODOMETRY IS COMPUTED (dead reckoning)
  Each new message gives the wheel velocities (v, w). Over a time step dt:
      x   += v * cos(theta + w*dt/2) * dt
      y   += v * sin(theta + w*dt/2) * dt
      theta += w * dt
  Small errors in v and w are integrated forever -> the estimate drifts without bound.

ONE OWNER PER TRANSFORM
  Exactly one node may publish odom -> base_footprint. In this project it is THIS node
  (publish_tf:=true), or the EKF when use_ekf:=true (then this node publishes
  /wheel/odom without TF and the EKF fuses it with the IMU).
"""
import math

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from ..common.geometry import normalize_angle, yaw_to_quaternion
from ..common.ros_utils import run_node, yaw_of


class OdometryProcessor(Node):
    def __init__(self):
        super().__init__('odometry_processor')
        p = self.declare_parameter
        self.input_topic = p('input_topic', '/wheel/odom_raw').value
        self.output_topic = p('output_topic', '/odom').value
        self.publish_tf = p('publish_tf', True).value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.mode = p('mode', 'integrate').value                 # integrate | passthrough
        self.lin_scale = p('linear_scale_error', 0.0).value
        self.ang_scale = p('angular_scale_error', 0.0).value
        self.lin_noise = p('linear_noise_std', 0.0).value       # m/s
        self.ang_noise = p('angular_noise_std', 0.0).value      # rad/s
        self.ang_bias = p('angular_bias', 0.0).value            # rad/s, applied only while moving
        seed = p('random_seed', 0).value
        self.rng = np.random.default_rng(seed if seed else None)

        self.pose = [0.0, 0.0, 0.0]
        self.last_stamp = None
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.pub = self.create_publisher(Odometry, self.output_topic, 20)
        self.create_subscription(Odometry, self.input_topic, self._on_raw, 20)
        self.get_logger().info(
            f'{self.input_topic} -> {self.output_topic} (mode={self.mode}, publish_tf={self.publish_tf}, '
            f'scale errors lin={self.lin_scale:+.3f} ang={self.ang_scale:+.3f}, '
            f'noise lin={self.lin_noise} ang={self.ang_noise}, bias={self.ang_bias})')

    def _on_raw(self, raw):
        t = raw.header.stamp.sec + raw.header.stamp.nanosec * 1e-9
        v = raw.twist.twist.linear.x
        w = raw.twist.twist.angular.z

        if self.mode == 'passthrough':
            # Use the simulator's odometry as-is (it is almost perfect).
            self.pose = [raw.pose.pose.position.x, raw.pose.pose.position.y,
                         yaw_of(raw.pose.pose.orientation)]
        else:
            if self.last_stamp is None or t <= self.last_stamp:
                self.last_stamp = t
                return
            dt = min(t - self.last_stamp, 0.2)     # guard against huge jumps (sim pause/reset)
            self.last_stamp = t
            moving = abs(v) > 1e-3 or abs(w) > 1e-3
            # --- the error model ---
            v_meas = v * (1.0 + self.lin_scale)
            w_meas = w * (1.0 + self.ang_scale)
            if moving:
                v_meas += self.rng.normal(0.0, self.lin_noise) if self.lin_noise > 0 else 0.0
                w_meas += self.rng.normal(0.0, self.ang_noise) if self.ang_noise > 0 else 0.0
                w_meas += self.ang_bias
            # --- dead reckoning (midpoint integration) ---
            x, y, th = self.pose
            x += v_meas * math.cos(th + 0.5 * w_meas * dt) * dt
            y += v_meas * math.sin(th + 0.5 * w_meas * dt) * dt
            th = normalize_angle(th + w_meas * dt)
            self.pose = [x, y, th]
            v, w = v_meas, w_meas

        qx, qy, qz, qw = yaw_to_quaternion(self.pose[2])
        out = Odometry()
        out.header.stamp = raw.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x, out.pose.pose.position.y = self.pose[0], self.pose[1]
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = qz, qw
        out.twist.twist.linear.x = v
        out.twist.twist.angular.z = w
        # Covariances: used by the EKF to weigh this source. Large values on z/roll/pitch
        # mean "I know nothing about these" (a 2D robot never measures them).
        pc = [0.0] * 36
        pc[0] = pc[7] = 0.01 + 4 * self.lin_scale ** 2
        pc[14] = pc[21] = pc[28] = 1e6
        pc[35] = 0.02 + 4 * self.ang_scale ** 2
        out.pose.covariance = pc
        tc = [0.0] * 36
        tc[0] = 0.002 + self.lin_noise ** 2
        tc[7] = 0.002
        tc[14] = tc[21] = tc[28] = 1e6
        tc[35] = 0.005 + self.ang_noise ** 2
        out.twist.covariance = tc
        self.pub.publish(out)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = out.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x, tf.transform.translation.y = self.pose[0], self.pose[1]
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    run_node(OdometryProcessor, args)


if __name__ == '__main__':
    main()
