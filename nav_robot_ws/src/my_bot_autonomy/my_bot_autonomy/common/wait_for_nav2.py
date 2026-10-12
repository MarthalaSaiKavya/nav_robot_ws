"""
wait_for_nav2 - a tiny "gate" process: exits as soon as Nav2 is ACTIVE.

    ros2 run my_bot_autonomy wait_for_nav2                       # waits for bt_navigator
    ros2 run my_bot_autonomy wait_for_nav2 --ros-args -p nodes:="['bt_navigator','behavior_server']"

WHY: Nav2's servers are lifecycle nodes. Their action servers EXIST from the start, but
they REJECT goals until the lifecycle manager has configured + activated them (a few
seconds). A node that sends a goal too early (e.g. explore_lite) gets "Goal was REJECTED".

HOW IT IS USED (stage_explore.launch.py, explorer:=explore_lite) - a launch-event pattern:

    gate = Node(executable='wait_for_nav2')
    RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=[explore_lite_node]))

i.e. "start explore_lite when the gate process exits". The same thing in a terminal:
    ros2 lifecycle get /bt_navigator      ->  active [3]

Optional `initial_spin` [rad]: turn on the spot once Nav2 is active, BEFORE exiting.
explore_lite drives to the CENTROID of a frontier. Right at the start the only frontier is
the ring around the robot - whose centroid is the robot itself - so explore_lite "reaches"
it instantly and stops ("All frontiers traversed"). One spin lets SLAM see the whole
visible room first, and the frontiers become real doorways and corners.
"""
import sys

import rclpy
from rclpy.node import Node

from .nav2_actions import Nav2Actions
from .ros_utils import ignore_sigint, spin_in_background


def main(args=None):
    rclpy.init(args=args)
    node = Node('wait_for_nav2')
    nodes = list(node.declare_parameter('nodes', ['bt_navigator', 'behavior_server']).value)
    timeout = node.declare_parameter('timeout', 300.0).value
    initial_spin = node.declare_parameter('initial_spin', 0.0).value
    executor = spin_in_background(node)
    ok = False
    try:
        node.get_logger().info(f'waiting until {nodes} are active ...')
        actions = Nav2Actions(node)
        ok = actions.wait_until_active(tuple(nodes), timeout=timeout)
        node.get_logger().info('Nav2 is active' if ok else f'Nav2 NOT active after {timeout} s')
        if ok and initial_spin:
            node.get_logger().info(f'initial spin of {initial_spin:.2f} rad to seed the map ...')
            actions.spin_in_place(initial_spin, 40)
    except KeyboardInterrupt:
        pass
    finally:
        ignore_sigint()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
