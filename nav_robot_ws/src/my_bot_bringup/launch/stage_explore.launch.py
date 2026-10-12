"""
STAGE 1 of the full mission (also Phase 8 when combined with the robot):
SLAM + Nav2 + an EXPLORER that maps the room autonomously, returns home and saves the map.

This file does NOT start the simulator - it expects robot.launch.py to be running
(phase8_exploration.launch.py and the supervisor take care of that).

When the explorer finishes it EXITS, and `on_exit=Shutdown()` below makes this whole
launch file shut down -> SLAM and Nav2 stop cleanly -> the supervisor moves on.

    explorer:=frontier      our frontier_explorer (strategies below)            (default)
    explorer:=explore_lite  the community explore_lite node + our watcher
    explorer:=reactive      wall following / random walk, no Nav2 at all

    explore_strategy:=nearest | largest | utility | information_gain | random
"""
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from my_bot_autonomy.launch_utils import NAV_ARGS, declare, include, resolve_run_dir, share, values

ARGS = {
    'slam_method': ('slam_toolbox_async', 'slam_toolbox_async | slam_toolbox_sync | cartographer | learning_mapper'),
    'explorer': ('frontier', 'frontier | explore_lite | reactive'),
    'explore_strategy': ('utility', 'nearest | largest | utility | information_gain | random'),
    'reactive_mode': ('wall_follow', 'wall_follow | random_walk'),
    'use_path_distance': ('false', 'Rank frontiers by planned path length instead of straight line'),
    'max_duration': ('900.0', 'Give up exploring after this many seconds'),
    'exit_on_done': ('true', 'The explorer node exits when exploration is finished'),
    'shutdown_on_done': ('true', 'Shut this whole launch down when the explorer exits'),
    'run_dir': ('', 'Output folder (default ~/nav_robot_runs/<timestamp>)'),
}


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ARGS) + list(NAV_ARGS))
    run_dir = resolve_run_dir(a['run_dir'])
    exit_on_done = a['exit_on_done'].lower() == 'true'
    explorer = a['explorer']
    shutdown = exit_on_done and a['shutdown_on_done'].lower() == 'true'
    on_exit = Shutdown(reason='exploration finished') if shutdown else None
    actions = [LogInfo(msg=f'[stage_explore] slam={a["slam_method"]} explorer={explorer} '
                           f'strategy={a["explore_strategy"]} -> {run_dir}'),
               include('my_bot_navigation', 'slam.launch.py', {'slam_method': a['slam_method']})]
    if explorer != 'reactive':
        actions.append(include('my_bot_navigation', 'navigation.launch.py', {k: a[k] for k in NAV_ARGS}))

    common = {'use_sim_time': True, 'run_dir': run_dir, 'exit_on_done': exit_on_done}
    if explorer == 'reactive':
        actions.append(Node(package='my_bot_autonomy', executable='reactive_explorer', name='reactive_explorer',
                            output='screen', on_exit=on_exit,
                            parameters=[common, {'mode': a['reactive_mode'],
                                                 'duration': float(a['max_duration'])}]))
    else:
        strategy = 'external' if explorer == 'explore_lite' else a['explore_strategy']
        actions.append(Node(package='my_bot_autonomy', executable='frontier_explorer', name='frontier_explorer',
                            output='screen', on_exit=on_exit,
                            parameters=[common, {'strategy': strategy,
                                                 'use_path_distance': a['use_path_distance'].lower() == 'true',
                                                 'max_duration': float(a['max_duration'])}]))
        if explorer == 'explore_lite':
            # explore_lite sends its first goal immediately; if Nav2 is not ACTIVE yet that goal
            # is rejected and explore_lite waits a whole progress_timeout before trying again.
            # And with an almost empty map its only frontier is the ring around the robot.
            # So: start a tiny "gate" process that waits until Nav2 is active, spins the robot
            # once (SLAM sees the room), then exits - and start explore_lite when the gate
            # exits (a launch EVENT HANDLER).
            gate = Node(package='my_bot_autonomy', executable='wait_for_nav2', name='wait_for_nav2',
                        output='screen', parameters=[{'use_sim_time': True, 'initial_spin': 6.28}])
            explore = Node(package='explore_lite', executable='explore', name='explore_node', output='screen',
                           parameters=[share('my_bot_bringup', 'config', 'explore_lite.yaml'),
                                       {'use_sim_time': True}])
            actions += [gate, RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=[explore]))]
    return actions


def generate_launch_description():
    return LaunchDescription(declare(ARGS) + declare(NAV_ARGS) + [OpaqueFunction(function=launch_setup)])
