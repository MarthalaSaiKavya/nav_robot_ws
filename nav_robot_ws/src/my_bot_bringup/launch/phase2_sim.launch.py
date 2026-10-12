"""
PHASE 2 - The robot in the simulated room, driven by keyboard.

    ros2 launch my_bot_bringup phase2_sim.launch.py                 # Gazebo
    ros2 launch my_bot_bringup phase2_sim.launch.py sim:=lite       # lightweight simulator
    ros2 launch my_bot_bringup phase2_sim.launch.py drive:=ros2_control

Then, in a SECOND terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

RViz (fixed frame = odom) shows the LiDAR, plus two trails:
    green  = ground truth (where the robot REALLY is)
    yellow = odometry (where the wheels THINK it is)  -> watch them separate (drift)!
Verify: ros2 topic hz /scan (~10 Hz), ros2 topic hz /odom, ros2 run tf2_tools view_frames
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction

from my_bot_autonomy.launch_utils import ROBOT_ARGS, declare, include, resolve_run_dir, rviz_node, values


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ROBOT_ARGS) + ['run_dir', 'rviz'])
    run_dir = resolve_run_dir(a['run_dir'])
    actions = [include('my_bot_bringup', 'robot.launch.py', {**{k: a[k] for k in ROBOT_ARGS}, 'run_dir': run_dir}),
               LogInfo(msg='Drive the robot from another terminal:  ros2 run teleop_twist_keyboard teleop_twist_keyboard')]
    if a['rviz'].lower() == 'true':
        actions.append(rviz_node('sim.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription(declare(ROBOT_ARGS) + [
        DeclareLaunchArgument('run_dir', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup)])
