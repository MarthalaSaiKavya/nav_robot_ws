"""
sim_world - poke the simulated world from the command line (works with Gazebo AND sim:=lite).

    ros2 run my_bot_autonomy sim_world spawn box1 1.5 -1.0                 # 0.5 x 0.5 x 0.6 m box
    ros2 run my_bot_autonomy sim_world spawn wall1 3.0 1.5 --size 1.4 0.4 0.8 --yaw 0.0
    ros2 run my_bot_autonomy sim_world remove box1
    ros2 run my_bot_autonomy sim_world teleport my_bot 1.0 -4.2 3.14       # "kidnap" the robot
    ... add --backend gz_cli to use the `gz service` command line instead of the ROS services
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from .ros_utils import ignore_sigint, spin_in_background
from .sim_services import SimWorldClient


def main(argv=None):
    argv = remove_ros_args(sys.argv if argv is None else argv)[1:]
    ap = argparse.ArgumentParser(prog='sim_world', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='nav_room')
    ap.add_argument('--backend', default='ros', choices=['ros', 'gz_cli'])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('spawn')
    s.add_argument('name')
    s.add_argument('x', type=float)
    s.add_argument('y', type=float)
    s.add_argument('--yaw', type=float, default=0.0)
    s.add_argument('--size', type=float, nargs=3, default=[0.5, 0.5, 0.6])
    r = sub.add_parser('remove')
    r.add_argument('name')
    t = sub.add_parser('teleport')
    t.add_argument('name')
    t.add_argument('x', type=float)
    t.add_argument('y', type=float)
    t.add_argument('yaw', type=float, nargs='?', default=0.0)
    args = ap.parse_args(argv)

    rclpy.init()
    node = Node('sim_world_cli')
    executor = spin_in_background(node)
    ok = False
    try:
        client = SimWorldClient(node, args.world, args.backend)
        if args.backend == 'ros' and not client.available(10.0):
            print(f'service /world/{args.world}/create not available - is the simulator running?')
        elif args.cmd == 'spawn':
            ok = client.spawn_box(args.name, args.x, args.y, args.yaw, *args.size)
        elif args.cmd == 'remove':
            ok = client.remove(args.name)
        else:
            ok = client.set_pose(args.name, args.x, args.y, args.yaw)
    finally:
        ignore_sigint()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
