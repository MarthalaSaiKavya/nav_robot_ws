"""
=========================================================================================
 PHASE 9 - THE SINGLE COMMAND.  Map -> save -> re-localize -> mission -> report, unattended.
=========================================================================================

    ros2 launch my_bot_bringup full_mission.launch.py
    ros2 launch my_bot_bringup full_mission.launch.py gui:=false rviz:=false     # fastest
    ros2 launch my_bot_bringup full_mission.launch.py sim:=lite                   # no Gazebo
    ros2 launch my_bot_bringup full_mission.launch.py explore_strategy:=nearest controller:=mppi

Starts: simulator + robot + odometry + evaluator (+ RViz) + the SUPERVISOR node, which runs
    stage 1  (stage_explore.launch.py)  explore + SLAM + save map
    stage 2  (stage_mission.launch.py)  AMCL from a rough guess + 5-goal mission with a surprise
                                        obstacle, a blocked doorway and a full "cage" block
and prints the report (also saved as <run_dir>/summary.md). The whole launch exits when done.
The number to beat: "Total wall-clock time (launch -> all goals complete)".

Every variant argument of the project can be passed here and is forwarded to the stages
(see my_bot_autonomy/launch_utils.py for the full list).
"""
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, Shutdown
from launch_ros.actions import Node

from my_bot_autonomy.launch_utils import NAV_ARGS, ROBOT_ARGS, declare, include, resolve_run_dir, rviz_node, share, values

STAGE_ARGS = {
    # stage 1
    'slam_method': ('slam_toolbox_async', 'slam_toolbox_async | slam_toolbox_sync | cartographer | learning_mapper'),
    'explorer': ('frontier', 'frontier | explore_lite | reactive'),
    'explore_strategy': ('utility', 'nearest | largest | utility | information_gain | random'),
    'reactive_mode': ('wall_follow', 'wall_follow | random_walk'),
    'max_duration': ('900.0', 'Exploration time limit [s]'),
    # stage 2
    'localization_method': ('amcl', 'amcl | slam_toolbox | learning_pf'),
    'amcl_profile': ('default', 'default | beam_model | many_particles | kidnap_recovery | fast'),
    'mission_file': ('demo_mission.yaml', 'Mission yaml'),
    'mission_method': ('navigate_to_pose', 'navigate_to_pose | follow_waypoints | through_poses | raw_action'),
    'initial_pose_mode': ('topic', 'topic | params | global | none'),
    'scenario_file': ('demo_scenario.yaml', 'Scenario yaml or none'),
    # pipeline
    'skip_exploration': ('false', 'true = skip stage 1 and use map:=...'),
    'map': ('', 'Map to use when skip_exploration:=true'),
    'run_dir': ('', 'Output folder (default ~/nav_robot_runs/<timestamp>)'),
    'rviz': ('true', 'Open RViz'),
    'stage_output': ('important', 'important | all  - how much of each stage output to print (full logs are in run_dir)'),
}


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ROBOT_ARGS) + list(NAV_ARGS) + list(STAGE_ARGS))
    run_dir = resolve_run_dir(a['run_dir'])
    nav = ' '.join(f'{k}:={a[k]}' for k in NAV_ARGS)
    explore_args = (f'slam_method:={a["slam_method"]} explorer:={a["explorer"]} '
                    f'explore_strategy:={a["explore_strategy"]} reactive_mode:={a["reactive_mode"]} '
                    f'max_duration:={a["max_duration"]} {nav}')
    mission_args = (f'localization_method:={a["localization_method"]} amcl_profile:={a["amcl_profile"]} '
                    f'mission_method:={a["mission_method"]} initial_pose_mode:={a["initial_pose_mode"]} '
                    f'scenario_file:={a["scenario_file"]} {nav}')
    world = a['world'] if a['world'].startswith('/') else share('my_bot_gazebo', 'worlds', a['world'])
    actions = [
        LogInfo(msg=f'[full_mission] run folder: {run_dir}'),
        include('my_bot_bringup', 'robot.launch.py', {**{k: a[k] for k in ROBOT_ARGS}, 'run_dir': run_dir}),
        Node(package='my_bot_autonomy', executable='supervisor', name='supervisor', output='screen',
             on_exit=Shutdown(reason='supervisor finished'),
             parameters=[{'use_sim_time': True, 'run_dir': run_dir, 'explore_args': explore_args,
                          'mission_args': mission_args, 'mission_file': a['mission_file'],
                          'skip_exploration': a['skip_exploration'].lower() == 'true', 'map': a['map'],
                          'world_file': world, 'stage_output': a['stage_output']}]),
    ]
    if a['rviz'].lower() == 'true':
        actions.append(rviz_node('navigation.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription(declare(ROBOT_ARGS) + declare(NAV_ARGS) + declare(STAGE_ARGS) +
                             [OpaqueFunction(function=launch_setup)])
