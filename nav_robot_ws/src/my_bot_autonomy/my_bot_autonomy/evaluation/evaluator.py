"""
evaluator - continuously MEASURES how well the robot knows where it is (simulation only).

Compares three poses every 0.2 s:
    truth     /ground_truth/odom                       (the simulator's "cheat")
    estimate  TF map -> base_footprint                 (SLAM / AMCL / your PF)
    odometry  TF odom -> base_footprint                (dead reckoning only)

Outputs:
  /evaluation/localization_error  std_msgs/Float32  |estimate - truth| in metres
  /evaluation/ground_truth_path, /evaluation/estimated_path, /evaluation/odom_path  (RViz trails)
  <run_dir>/pose_log.csv          every sample (plot it! see `run_report`)
  <run_dir>/evaluation_summary.json at shutdown: mean / p95 / max errors, near-collisions

FRAMES: in this project map == world (the robot starts SLAM at the world origin).
If you start SLAM somewhere else, set map_origin_in_world to that start pose.
Odometry has no fixed relation to the world, so it is aligned to the truth on the first
sample (and again after a teleport) - the error you see is pure DRIFT since then.

NEAR-COLLISIONS: a LiDAR beam shorter than the distance from the LiDAR to the robot's own
outline (+ margin) means something is touching / almost touching the robot body.
"""
import csv
import math
import os

import numpy as np
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener

from ..common.geometry import compose, inverse, normalize_angle
from ..common.ros_utils import lookup_pose_2d, make_pose_stamped, make_run_dir, run_node, write_json, yaw_of


class Evaluator(Node):
    def __init__(self):
        super().__init__('evaluator')
        p = self.declare_parameter
        self.run_dir = make_run_dir(p('run_dir', '').value)
        self.map_in_world = tuple(p('map_origin_in_world', [0.0, 0.0, 0.0]).value)
        # robot outline relative to base_link (chassis + wheels), LiDAR offset
        self.half_length = p('footprint_half_length', 0.18).value
        self.half_width = p('footprint_half_width', 0.195).value
        self.lidar_x = p('lidar_offset_x', 0.05).value
        self.contact_margin = p('contact_margin', 0.03).value
        self.truth = None
        self.odom_align = None          # world <- odom transform found by alignment
        self.samples = []
        self.near_collisions = 0
        self._in_contact = False
        self._body_dist = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 20)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.err_pub = self.create_publisher(Float32, '/evaluation/localization_error', 10)
        self.paths = {k: Path() for k in ('ground_truth', 'estimated', 'odom')}
        self.path_pubs = {k: self.create_publisher(Path, f'/evaluation/{k}_path', 1) for k in self.paths}
        self.csv_file = open(os.path.join(self.run_dir, 'pose_log.csv'), 'w', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['t', 'truth_x', 'truth_y', 'truth_yaw', 'est_x', 'est_y', 'est_yaw',
                           'odom_x', 'odom_y', 'odom_yaw', 'loc_error_m', 'loc_error_yaw', 'odom_drift_m'])
        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_paths)
        self.create_timer(5.0, self._save)          # save often: a killed process still leaves results
        self.get_logger().info(f'evaluating -> {self.run_dir}/pose_log.csv')

    def _on_truth(self, msg):
        p = msg.pose.pose
        new = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.truth is not None and math.hypot(new[0] - self.truth[0], new[1] - self.truth[1]) > 0.5:
            self.get_logger().info('teleport detected -> re-aligning odometry drift measurement')
            self.odom_align = None
        self.truth = new

    def _on_scan(self, msg):
        if self._body_dist is None or len(self._body_dist) != len(msg.ranges):
            # distance from the LiDAR to the robot outline along each beam (computed once)
            ang = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
            c, s = np.cos(ang), np.sin(ang)
            with np.errstate(divide='ignore'):
                tx = np.where(c > 0, (self.half_length - self.lidar_x) / c,
                              np.where(c < 0, (-self.half_length - self.lidar_x) / c, np.inf))
                ty = np.where(np.abs(s) > 0, self.half_width / np.abs(s), np.inf)
            self._body_dist = np.minimum(tx, ty)
        r = np.asarray(msg.ranges, dtype=float)
        contact = bool(np.any(np.isfinite(r) & (r < self._body_dist + self.contact_margin)
                              & (r >= msg.range_min)))
        if contact and not self._in_contact:
            self.near_collisions += 1
            self.get_logger().warn(f'near-collision #{self.near_collisions}: obstacle within '
                                   f'{self.contact_margin * 100:.0f} cm of the robot body')
        self._in_contact = contact

    def _tick(self):
        if self.truth is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        est_map = lookup_pose_2d(self.tf_buffer, 'map', 'base_footprint')
        odom = lookup_pose_2d(self.tf_buffer, 'odom', 'base_footprint')
        est = compose(self.map_in_world, est_map) if est_map else None
        row = [round(t, 3), *[round(v, 4) for v in self.truth]]
        row += [round(v, 4) for v in est] if est else ['', '', '']
        loc_err = yaw_err = drift = None
        if est:
            loc_err = math.hypot(est[0] - self.truth[0], est[1] - self.truth[1])
            yaw_err = abs(normalize_angle(est[2] - self.truth[2]))
            self.err_pub.publish(Float32(data=float(loc_err)))
        if odom:
            if self.odom_align is None:
                self.odom_align = compose(self.truth, inverse(odom))   # world <- odom
            o = compose(self.odom_align, odom)
            drift = math.hypot(o[0] - self.truth[0], o[1] - self.truth[1])
            row += [round(v, 4) for v in o]
        else:
            row += ['', '', '']
        row += [round(loc_err, 4) if loc_err is not None else '', round(yaw_err, 4) if yaw_err is not None else '',
                round(drift, 4) if drift is not None else '']
        self.csv.writerow(row)
        self.samples.append((t, loc_err, yaw_err, drift))
        stamp = self.get_clock().now().to_msg()
        self._append('ground_truth', self.truth, stamp)
        if est:
            self._append('estimated', est, stamp)
        if odom:
            self._append('odom', compose(self.odom_align, odom), stamp)

    def _append(self, key, pose, stamp):
        path = self.paths[key]
        # world == map in this project, so all trails are drawn in the map frame
        ps = make_pose_stamped(*compose(inverse(self.map_in_world), pose), 'map', stamp)
        if path.poses and math.hypot(ps.pose.position.x - path.poses[-1].pose.position.x,
                                     ps.pose.position.y - path.poses[-1].pose.position.y) < 0.05:
            return
        path.poses.append(ps)
        if len(path.poses) > 5000:
            path.poses = path.poses[-5000:]

    def _publish_paths(self):
        stamp = self.get_clock().now().to_msg()
        for k, path in self.paths.items():
            path.header.frame_id = 'map'
            path.header.stamp = stamp
            self.path_pubs[k].publish(path)

    def summary(self):
        def stats(vals):
            vals = [v for v in vals if v is not None]
            if not vals:
                return None
            a = np.array(vals)
            return {'mean': round(float(a.mean()), 4), 'p95': round(float(np.percentile(a, 95)), 4),
                    'max': round(float(a.max()), 4), 'final': round(float(a[-1]), 4)}
        return {'samples': len(self.samples),
                'localization_error_m': stats([s[1] for s in self.samples]),
                'localization_error_yaw_rad': stats([s[2] for s in self.samples]),
                'odometry_drift_m': stats([s[3] for s in self.samples]),
                'near_collisions': self.near_collisions}

    def _save(self):
        try:
            self.csv_file.flush()
            write_json(os.path.join(self.run_dir, 'evaluation_summary.json'), self.summary())
        except Exception as e:  # never crash the evaluator because of disk issues
            self.get_logger().warn(f'could not save evaluation: {e}')

    def destroy_node(self):
        self._save()
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    run_node(Evaluator, args)


if __name__ == '__main__':
    main()
