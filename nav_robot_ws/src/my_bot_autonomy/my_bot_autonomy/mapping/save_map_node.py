"""
save_map - command line map saver (handy in Phase 3 after driving around by keyboard).

    ros2 run my_bot_autonomy save_map --ros-args -p output:=~/maps/room
    ros2 run my_bot_autonomy save_map --ros-args -p output:=~/maps/room -p methods:="['python']"
    ros2 run my_bot_autonomy save_map --ros-args -p output:=~/maps/room -p serialize:=true

The classic one-liner does the same as method 'nav2':
    ros2 run nav2_map_server map_saver_cli -f ~/maps/room
"""
import os
import rclpy
from rclpy.node import Node

from ..common.ros_utils import ignore_sigint, spin_in_background
from .map_saving import save_map_with_fallbacks, serialize_pose_graph


class SaveMapNode(Node):
    def __init__(self):
        super().__init__('save_map')
        self.output = os.path.expanduser(self.declare_parameter('output', '~/maps/room').value)
        self.methods = list(self.declare_parameter('methods', ['nav2', 'slam_toolbox', 'python']).value)
        self.serialize = self.declare_parameter('serialize', True).value


def main(args=None):
    rclpy.init(args=args)
    node = SaveMapNode()
    executor = spin_in_background(node)
    try:
        method = save_map_with_fallbacks(node, node.output, node.methods)
        if node.serialize:
            ok = serialize_pose_graph(node, node.output)
            node.get_logger().info(f'pose graph serialized: {ok} ({node.output}.posegraph/.data)'
                                   if ok else 'pose graph NOT serialized (is slam_toolbox running?)')
        print(f'\nResult: {"saved with " + method if method else "FAILED"} -> {node.output}.yaml\n')
    finally:
        ignore_sigint()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
