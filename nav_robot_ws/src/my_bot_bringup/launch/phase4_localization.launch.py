"""
PHASE 4 - Localize on a saved map from a ROUGH initial guess.

    ros2 launch my_bot_bringup phase4_localization.launch.py                    # reference map, AMCL
    ros2 launch my_bot_bringup phase4_localization.launch.py map:=$HOME/maps/my_room.yaml
    ros2 launch my_bot_bringup phase4_localization.launch.py localization_method:=learning_pf
    ros2 launch my_bot_bringup phase4_localization.launch.py amcl_profile:=beam_model

The robot spawns at (x, y, yaw) [default 0,0,0] but the localizer is told it is at
(init_x, init_y, init_yaw) [default 0.3, -0.2, 0.2] - deliberately wrong.
Drive a little (teleop) and watch the particle cloud collapse onto the true pose.
The evaluator prints the localization error (/evaluation/localization_error).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from my_bot_autonomy.launch_utils import ROBOT_ARGS, declare, include, resolve_run_dir, rviz_node, share, values

LOC_ARGS = {
    'localization_method': ('amcl', 'amcl | slam_toolbox | learning_pf'),
    'map': (share('my_bot_navigation', 'maps', 'nav_room_reference.yaml'), 'Map yaml'),
    'amcl_profile': ('default', 'default | beam_model | many_particles | kidnap_recovery | fast'),
    'set_initial_pose': ('true', 'Give the localizer the (rough) initial guess below'),
    'init_x': ('0.3', 'Initial guess x'), 'init_y': ('-0.2', 'Initial guess y'), 'init_yaw': ('0.2', 'Initial guess yaw'),
}


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ROBOT_ARGS) + list(LOC_ARGS) + ['run_dir', 'rviz'])
    run_dir = resolve_run_dir(a['run_dir'])
    actions = [include('my_bot_bringup', 'robot.launch.py', {**{k: a[k] for k in ROBOT_ARGS}, 'run_dir': run_dir}),
               include('my_bot_navigation', 'localization.launch.py', {
                   'localization_method': a['localization_method'], 'map': a['map'], 'amcl_profile': a['amcl_profile'],
                   'set_initial_pose': a['set_initial_pose'], 'x': a['init_x'], 'y': a['init_y'], 'yaw': a['init_yaw']})]
    if a['rviz'].lower() == 'true':
        actions.append(rviz_node('navigation.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription(declare(ROBOT_ARGS) + declare(LOC_ARGS) + [
        DeclareLaunchArgument('run_dir', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup)])
