"""
PHASE 8 - Autonomous exploration: the robot maps the unknown room by itself.

    ros2 launch my_bot_bringup phase8_exploration.launch.py
    ros2 launch my_bot_bringup phase8_exploration.launch.py explore_strategy:=nearest
    ros2 launch my_bot_bringup phase8_exploration.launch.py explorer:=reactive reactive_mode:=wall_follow
    ros2 launch my_bot_bringup phase8_exploration.launch.py explorer:=explore_lite      # build it first (README)
    ros2 launch my_bot_bringup phase8_exploration.launch.py slam_method:=cartographer

Watch in RViz: blue dots = frontiers, green = the chosen one, red discs = blacklisted spots.
When done the robot drives home, saves <run_dir>/explored_map.{yaml,pgm,posegraph,data}
and the launch shuts down. Then:
    ros2 run my_bot_autonomy compare_maps --map <run_dir>/explored_map.yaml --world nav_room.sdf
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from my_bot_autonomy.launch_utils import NAV_ARGS, ROBOT_ARGS, declare, include, resolve_run_dir, rviz_node, values

EXPLORE_ARGS = {
    'slam_method': ('slam_toolbox_async', 'slam_toolbox_async | slam_toolbox_sync | cartographer | learning_mapper'),
    'explorer': ('frontier', 'frontier | explore_lite | reactive'),
    'explore_strategy': ('utility', 'nearest | largest | utility | information_gain | random'),
    'reactive_mode': ('wall_follow', 'wall_follow | random_walk'),
    'use_path_distance': ('false', 'Rank frontiers by planned path length'),
    'max_duration': ('900.0', 'Exploration time limit [s]'),
}


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ROBOT_ARGS) + list(NAV_ARGS) + list(EXPLORE_ARGS) + ['run_dir', 'rviz'])
    run_dir = resolve_run_dir(a['run_dir'])
    actions = [include('my_bot_bringup', 'robot.launch.py', {**{k: a[k] for k in ROBOT_ARGS}, 'run_dir': run_dir}),
               include('my_bot_bringup', 'stage_explore.launch.py',
                       {**{k: a[k] for k in NAV_ARGS}, **{k: a[k] for k in EXPLORE_ARGS}, 'run_dir': run_dir})]
    if a['rviz'].lower() == 'true':
        actions.append(rviz_node('navigation.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription(declare(ROBOT_ARGS) + declare(NAV_ARGS) + declare(EXPLORE_ARGS) + [
        DeclareLaunchArgument('run_dir', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup)])
