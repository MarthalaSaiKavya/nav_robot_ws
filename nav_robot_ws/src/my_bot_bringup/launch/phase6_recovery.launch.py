"""
PHASE 6 - Recovery behaviours: the path gets FULLY blocked (a cage closes around the robot).

    ros2 launch my_bot_bringup phase6_recovery.launch.py
    ros2 launch my_bot_bringup phase6_recovery.launch.py bt:=patient        # waits longer -> succeeds
    ros2 launch my_bot_bringup phase6_recovery.launch.py bt:=no_recovery    # gives up immediately

What happens: one long goal (zone A -> zone B). 2 s after it starts, 4 walls appear around
the robot. The planner fails ("no valid path"), the behaviour tree clears the costmaps,
spins, waits, backs up ... (watch the terminal: "Running Spin", "Running Wait" ...).
The cage opens after 32 s (config/scenarios/hard_block_only.yaml). Depending on the tree,
Nav2 either is still trying and succeeds, or has already ABORTED -> the mission runner
retries once. Results: <run_dir>/mission_results.csv
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from my_bot_autonomy.launch_utils import NAV_ARGS, ROBOT_ARGS, declare, include, resolve_run_dir, rviz_node, values

OVERRIDES = {'x': '1.0', 'y': '-4.2', 'yaw': '3.14'}     # = recovery_test_mission start_pose


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ROBOT_ARGS) + list(NAV_ARGS) + ['run_dir', 'rviz', 'scenario_file'])
    run_dir = resolve_run_dir(a['run_dir'])
    actions = [include('my_bot_bringup', 'robot.launch.py', {**{k: a[k] for k in ROBOT_ARGS}, 'run_dir': run_dir}),
               include('my_bot_bringup', 'stage_mission.launch.py', {
                   **{k: a[k] for k in NAV_ARGS}, 'run_dir': run_dir, 'mission_file': 'recovery_test_mission.yaml',
                   'scenario_file': a['scenario_file'], 'initial_pose_mode': 'params', 'pre_mission_spin': 'false'})]
    if a['rviz'].lower() == 'true':
        actions.append(rviz_node('navigation.rviz'))
    return actions


def generate_launch_description():
    robot_args = {k: (OVERRIDES.get(k, v[0]), v[1]) for k, v in ROBOT_ARGS.items()}
    return LaunchDescription(declare(robot_args) + declare(NAV_ARGS) + [
        DeclareLaunchArgument('scenario_file', default_value='hard_block_only.yaml'),
        DeclareLaunchArgument('run_dir', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup)])
