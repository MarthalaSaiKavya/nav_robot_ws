"""
PHASE 7 - The queued multi-goal mission on a saved map (with surprise obstacle, re-route,
hard block) - everything except the autonomous mapping.

    ros2 launch my_bot_bringup phase7_mission.launch.py
    ros2 launch my_bot_bringup phase7_mission.launch.py map:=~/nav_robot_runs/<run>/explored_map.yaml
    ros2 launch my_bot_bringup phase7_mission.launch.py mission_method:=follow_waypoints scenario_file:=none
    ros2 launch my_bot_bringup phase7_mission.launch.py mission_file:=patrol_mission.yaml controller:=mppi

The robot spawns at the demo mission's start pose (1.0, -4.2, 3.14); the localizer gets a
rough guess. Results land in the run folder: mission_results.csv/json, pose_log.csv.
Afterwards:  ros2 run my_bot_autonomy run_report
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from my_bot_autonomy.launch_utils import NAV_ARGS, ROBOT_ARGS, declare, include, resolve_run_dir, rviz_node, share, values

OVERRIDES = {'x': '1.0', 'y': '-4.2', 'yaw': '3.14'}     # = demo_mission start_pose
MISSION_ARGS = {
    'localization_method': ('amcl', 'amcl | slam_toolbox | learning_pf'),
    'map': (share('my_bot_navigation', 'maps', 'nav_room_reference.yaml'), 'Map yaml'),
    'amcl_profile': ('default', 'AMCL profile'),
    'mission_file': ('demo_mission.yaml', 'Mission yaml'),
    'mission_method': ('navigate_to_pose', 'navigate_to_pose | follow_waypoints | through_poses | raw_action'),
    'initial_pose_mode': ('topic', 'topic | params | global | none'),
    'scenario_file': ('demo_scenario.yaml', 'Scenario yaml or none'),
}


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ROBOT_ARGS) + list(NAV_ARGS) + list(MISSION_ARGS) + ['run_dir', 'rviz'])
    run_dir = resolve_run_dir(a['run_dir'])
    actions = [include('my_bot_bringup', 'robot.launch.py', {**{k: a[k] for k in ROBOT_ARGS}, 'run_dir': run_dir}),
               include('my_bot_bringup', 'stage_mission.launch.py',
                       {**{k: a[k] for k in NAV_ARGS}, **{k: a[k] for k in MISSION_ARGS}, 'run_dir': run_dir})]
    if a['rviz'].lower() == 'true':
        actions.append(rviz_node('navigation.rviz'))
    return actions


def generate_launch_description():
    robot_args = {k: (OVERRIDES.get(k, v[0]), v[1]) for k, v in ROBOT_ARGS.items()}
    return LaunchDescription(declare(robot_args) + declare(NAV_ARGS) + declare(MISSION_ARGS) + [
        DeclareLaunchArgument('run_dir', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup)])
