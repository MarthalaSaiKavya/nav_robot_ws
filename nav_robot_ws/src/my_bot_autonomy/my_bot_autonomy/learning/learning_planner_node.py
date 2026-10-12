"""
learning_planner - our A* / Dijkstra / greedy planner (learning/grid_planners.py) as a ROS node.

    ros2 launch my_bot_bringup learning_stack.launch.py              # full "own Nav2"
    ros2 run my_bot_autonomy learning_planner --ros-args -p algorithm:=dijkstra -p use_sim_time:=true

Click "2D Goal Pose" in RViz (or publish on goal_topic). The node plans from the robot's
current pose (TF map -> base_footprint) and publishes:
   /learning/plan            nav_msgs/Path            (orange line in RViz)
   /learning/search_markers  every EXPANDED cell     (see how much work each algorithm does!)
With compare_all:=true it also runs the other algorithms and logs a comparison table.

Tip: run it NEXT TO Nav2 with goal_topic:=/goal_pose: the same click then produces Nav2's
plan (/plan, red) and ours (/learning/plan, orange) side by side.
"""
import time

import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from ..common.map_io import grid_from_msg
from ..common.ros_utils import lookup_pose_2d, make_pose_stamped, run_node
from .grid_planners import ALGORITHMS, build_cost_grid, nearest_free, path_length, plan, shortcut_smooth


class LearningPlanner(Node):
    def __init__(self):
        super().__init__('learning_planner')
        p = self.declare_parameter
        self.algorithm = p('algorithm', 'astar').value
        self.robot_radius = p('robot_radius', 0.24).value
        self.inflation_weight = p('inflation_weight', 5.0).value
        self.smooth = p('smooth', True).value
        self.compare_all = p('compare_all', True).value
        goal_topic = p('goal_topic', '/learning/goal').value
        self.cost = None
        self.res = self.origin = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, p('map_topic', '/map').value, self._on_map, map_qos)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)
        self.path_pub = self.create_publisher(Path, '/learning/plan', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/learning/search_markers', 1)
        self.get_logger().info(f'learning_planner ({self.algorithm}) waiting for goals on {goal_topic}')

    def _on_map(self, msg):
        grid, self.res, self.origin = grid_from_msg(msg)
        self.cost = build_cost_grid(grid, self.res, self.robot_radius, self.inflation_weight)
        self.get_logger().info('cost grid ready (obstacles inflated by the robot radius)')

    def _cell(self, x, y):
        return (int((y - self.origin[1]) / self.res), int((x - self.origin[0]) / self.res))

    def _world(self, r, c):
        return (self.origin[0] + (c + 0.5) * self.res, self.origin[1] + (r + 0.5) * self.res)

    def _on_goal(self, msg):
        if self.cost is None:
            self.get_logger().warn('no map yet')
            return
        robot = lookup_pose_2d(self.tf_buffer, 'map', 'base_footprint')
        if robot is None:
            self.get_logger().warn('robot pose unknown (no map -> base_footprint TF)')
            return
        start = nearest_free(self.cost, self._cell(robot[0], robot[1]))
        goal = nearest_free(self.cost, self._cell(msg.pose.position.x, msg.pose.position.y))
        if start is None or goal is None:
            self.get_logger().warn('start or goal is inside an obstacle')
            return
        algos = ALGORITHMS if self.compare_all else (self.algorithm,)
        chosen = None
        report = []
        for alg in algos:
            t0 = time.time()
            path, n_exp, expanded = plan(self.cost, start, goal, alg)
            dt = (time.time() - t0) * 1000
            length = path_length(path, self.res) if path else float('nan')
            report.append(f'{alg:9s} expanded {n_exp:6d} cells  {dt:7.1f} ms  length {length:5.2f} m')
            if alg == self.algorithm:
                chosen = (path, expanded)
        self.get_logger().info('planner comparison:\n  ' + '\n  '.join(report))
        path, expanded = chosen
        if path is None:
            self.get_logger().warn('NO PATH found')
            return
        if self.smooth:
            path = shortcut_smooth(path, self.cost)
        out = Path()
        out.header.frame_id = 'map'
        out.header.stamp = self.get_clock().now().to_msg()
        for r, c in path:
            x, y = self._world(r, c)
            out.poses.append(make_pose_stamped(x, y, 0.0, 'map', out.header.stamp))
        out.poses[-1].pose.orientation = msg.pose.orientation
        self.path_pub.publish(out)
        self._publish_expanded(expanded)

    def _publish_expanded(self, expanded):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns, m.id, m.type = 'expanded', 0, Marker.POINTS
        m.scale.x = m.scale.y = self.res * 0.6
        m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.3, 1.0, 0.5
        for r, c in np.argwhere(expanded)[::2]:
            x, y = self._world(r, c)
            m.points.append(Point(x=x, y=y, z=0.01))
        self.marker_pub.publish(MarkerArray(markers=[m]))


def main(args=None):
    run_node(LearningPlanner, args)


if __name__ == '__main__':
    main()
