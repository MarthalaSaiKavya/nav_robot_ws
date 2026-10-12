"""
PHASE 3 - SLAM while you drive by keyboard, then save the map.

    ros2 launch my_bot_bringup phase3_slam.launch.py
    ros2 launch my_bot_bringup phase3_slam.launch.py slam_method:=cartographer
    ros2 launch my_bot_bringup phase3_slam.launch.py slam_method:=slam_toolbox_sync odometry_profile:=terrible

Terminal 2:  ros2 run teleop_twist_keyboard teleop_twist_keyboard     (drive slowly, <= 0.3 m/s,
             visit every corner, come back to the start to trigger a LOOP CLOSURE)
Terminal 3:  ros2 run my_bot_autonomy save_map --ros-args -p output:=$HOME/maps/my_room
             ros2 run my_bot_autonomy compare_maps --map $HOME/maps/my_room.yaml --world nav_room.sdf
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction

from my_bot_autonomy.launch_utils import ROBOT_ARGS, declare, include, resolve_run_dir, rviz_node, values


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ROBOT_ARGS) + ['run_dir', 'rviz', 'slam_method'])
    run_dir = resolve_run_dir(a['run_dir'])
    actions = [include('my_bot_bringup', 'robot.launch.py', {**{k: a[k] for k in ROBOT_ARGS}, 'run_dir': run_dir}),
               include('my_bot_navigation', 'slam.launch.py', {'slam_method': a['slam_method']}),
               LogInfo(msg='Drive: ros2 run teleop_twist_keyboard teleop_twist_keyboard | '
                           'Save: ros2 run my_bot_autonomy save_map --ros-args -p output:=$HOME/maps/my_room')]
    if a['rviz'].lower() == 'true':
        actions.append(rviz_node('navigation.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription(declare(ROBOT_ARGS) + [
        DeclareLaunchArgument('slam_method', default_value='slam_toolbox_async',
                              description='slam_toolbox_async | slam_toolbox_sync | cartographer | learning_mapper'),
        DeclareLaunchArgument('run_dir', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup)])
