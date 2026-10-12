"""
STAGE 2 of the full mission (also Phase 7 when combined with the robot):
localization on the saved map + Nav2 + mission runner + scenario director.

Expects robot.launch.py to be running. When the mission runner exits (all goals done or
given up), `on_exit=Shutdown()` stops this launch.

    ros2 launch my_bot_bringup stage_mission.launch.py map:=~/nav_robot_runs/<run>/explored_map.yaml

How the localizer gets its initial pose (initial_pose_mode):
    topic   mission_runner publishes the mission's ROUGH guess on /initialpose  (default)
    params  the guess is written into the localizer's parameters at start-up
    global  AMCL spreads particles everywhere; the robot spins to find itself
    none    (used automatically for localization_method:=slam_toolbox: guess via map_start_pose)
"""
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, Shutdown
from launch_ros.actions import Node

from my_bot_autonomy.launch_utils import NAV_ARGS, declare, include, resolve_run_dir, share, values
from my_bot_autonomy.mission.mission_io import load_mission

ARGS = {
    'localization_method': ('amcl', 'amcl | slam_toolbox | learning_pf'),
    'map': (share('my_bot_navigation', 'maps', 'nav_room_reference.yaml'), 'Map yaml to localize in'),
    'amcl_profile': ('default', 'default | beam_model | many_particles | kidnap_recovery | fast'),
    'mission_file': ('demo_mission.yaml', 'Mission yaml (name in my_bot_bringup/config/missions or path)'),
    'mission_method': ('navigate_to_pose', 'navigate_to_pose | follow_waypoints | through_poses | raw_action'),
    'initial_pose_mode': ('topic', 'topic | params | global | none'),
    'scenario_file': ('demo_scenario.yaml', 'Scenario yaml or "none"'),
    'pre_mission_spin': ('true', 'Spin once before the mission so the localizer converges'),
    'exit_on_done': ('true', 'Shut this launch down when the mission is finished'),
    'run_dir': ('', 'Output folder (default ~/nav_robot_runs/<timestamp>)'),
}


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ARGS) + list(NAV_ARGS))
    run_dir = resolve_run_dir(a['run_dir'])
    mission = load_mission(a['mission_file'])
    mode = a['initial_pose_mode']
    if a['localization_method'] == 'slam_toolbox' and mode == 'topic':
        mode = 'none'          # slam_toolbox localization takes its guess from map_start_pose
    gx, gy, gyaw = mission['initial_guess']
    set_init = mode in ('params', 'none')
    exit_on_done = a['exit_on_done'].lower() == 'true'
    actions = [
        LogInfo(msg=f'[stage_mission] localization={a["localization_method"]} map={a["map"]} '
                    f'mission={mission["name"]} method={a["mission_method"]} initial_pose_mode={mode}'),
        include('my_bot_navigation', 'localization.launch.py', {
            'localization_method': a['localization_method'], 'map': a['map'], 'amcl_profile': a['amcl_profile'],
            'set_initial_pose': str(set_init).lower(), 'x': str(gx), 'y': str(gy), 'yaw': str(gyaw)}),
        include('my_bot_navigation', 'navigation.launch.py', {k: a[k] for k in NAV_ARGS}),
        # NOTE: no name=... here: that would rename EVERY node in the process (mission_runner also
        # creates a BasicNavigator node) to the same name. Parameters then apply to all its nodes.
        Node(package='my_bot_autonomy', executable='mission_runner', output='screen',
             on_exit=Shutdown(reason='mission finished') if exit_on_done else None,
             parameters=[{'use_sim_time': True, 'mission_file': a['mission_file'], 'method': a['mission_method'],
                          'initial_pose_mode': mode, 'run_dir': run_dir,
                          'pre_mission_spin': a['pre_mission_spin'].lower() == 'true'}]),
    ]
    if a['scenario_file'] not in ('none', ''):
        actions.append(Node(package='my_bot_autonomy', executable='scenario_director', name='scenario_director',
                            output='screen',
                            parameters=[{'use_sim_time': True, 'scenario_file': a['scenario_file']}]))
    return actions


def generate_launch_description():
    return LaunchDescription(declare(ARGS) + declare(NAV_ARGS) + [OpaqueFunction(function=launch_setup)])
