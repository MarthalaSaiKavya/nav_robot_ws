"""
PHASE 9, ALTERNATIVE TECHNIQUE - the same unattended pipeline built purely from LAUNCH EVENTS
(no supervisor node).

    ros2 launch my_bot_bringup full_mission_events.launch.py

ROS 2 launch files are not just lists of nodes - they are event-driven programs:
  * OnProcessExit(...)          : "when process X exits, do Y"
  * EmitEvent(ShutdownProcess)  : stop selected processes
  * TimerAction(period, ...)    : do something later
  * OnShutdown(...)             : last words when everything stops

Pipeline:
  robot + stage_explore (shutdown_on_done:=false)
     --(frontier_explorer / reactive_explorer process exits)-->
  stop every stage-1 process (SLAM, Nav2 ...) + teleport the robot (sim_world CLI)
     --(8 s later)--> stage_mission (its mission_runner exit shuts everything down)
     --(OnShutdown)--> write run_summary.json + summary.md

Compare with full_mission.launch.py (supervisor node): the event version needs no extra
node, but the logic is spread over callbacks and harder to debug - a real design trade-off.
"""
import json
import os
import time

from launch import LaunchDescription
from launch.actions import (EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events.process import ShutdownProcess

from my_bot_autonomy.launch_utils import NAV_ARGS, ROBOT_ARGS, declare, include, resolve_run_dir, rviz_node, share, values
from my_bot_autonomy.mission.mission_io import load_mission

# Executables that belong to stage 1 (stopped when exploration ends)
STAGE1 = {'async_slam_toolbox_node', 'sync_slam_toolbox_node', 'cartographer_node', 'cartographer_occupancy_grid_node',
          'learning_mapper', 'map_saver_server', 'lifecycle_manager', 'controller_server', 'smoother_server',
          'planner_server', 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother',
          'collision_monitor', 'explore', 'frontier_explorer', 'reactive_explorer'}
EXPLORERS = ('frontier_explorer', 'reactive_explorer')

ARGS = {
    'slam_method': ('slam_toolbox_async', 'SLAM implementation'),
    'explorer': ('frontier', 'frontier | explore_lite | reactive'),
    'explore_strategy': ('utility', 'frontier strategy'),
    'mission_file': ('demo_mission.yaml', 'Mission yaml'),
    'mission_method': ('navigate_to_pose', 'mission method'),
    'scenario_file': ('demo_scenario.yaml', 'Scenario yaml or none'),
    'run_dir': ('', 'Output folder'),
    'rviz': ('true', 'Open RViz'),
}


def _is_stage1(action):
    try:
        return os.path.basename(action.process_details['cmd'][0]) in STAGE1
    except Exception:            # process not started / no details yet
        return False


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ROBOT_ARGS) + list(NAV_ARGS) + list(ARGS))
    run_dir = resolve_run_dir(a['run_dir'])
    t0 = time.time()
    map_yaml = os.path.join(run_dir, 'explored_map.yaml')
    sx, sy, syaw = load_mission(a['mission_file'])['start_pose']
    nav = {k: a[k] for k in NAV_ARGS}
    world = a['world'] if a['world'].startswith('/') else share('my_bot_gazebo', 'worlds', a['world'])

    stage2 = include('my_bot_bringup', 'stage_mission.launch.py', {
        **nav, 'map': map_yaml, 'mission_file': a['mission_file'], 'mission_method': a['mission_method'],
        'scenario_file': a['scenario_file'], 'run_dir': run_dir})

    def on_explorer_exit(event, ctx):
        if not event.process_name.startswith(EXPLORERS):
            return None          # some other process exited - not our trigger
        return [
            LogInfo(msg=f'[events] {event.process_name} finished -> stopping stage 1, teleporting, starting stage 2'),
            EmitEvent(event=ShutdownProcess(process_matcher=_is_stage1)),
            ExecuteProcess(cmd=['ros2', 'run', 'my_bot_autonomy', 'sim_world', 'teleport', 'my_bot',
                                str(sx), str(sy), str(syaw)], output='screen'),
            TimerAction(period=8.0, actions=[stage2]),
        ]

    def on_shutdown(event, ctx):
        mission = {}
        p = os.path.join(run_dir, 'mission_results.json')
        if os.path.exists(p):
            with open(p) as f:
                mission = json.load(f).get('summary', {})
        summary = {'run_dir': run_dir, 'technique': 'launch_events', 'mission': mission,
                   'success': bool(mission.get('all_goals_complete')),
                   'total_wall_clock_s': round(time.time() - t0, 1), 'map_yaml': map_yaml}
        with open(os.path.join(run_dir, 'run_summary.json'), 'w') as f:
            json.dump(summary, f, indent=2)
        try:   # score the explored map against the true world (same as the supervisor does)
            from my_bot_autonomy.common.map_io import read_map
            from my_bot_autonomy.common.world_model import World2D
            from my_bot_autonomy.mapping.compare_maps import compare
            from my_bot_autonomy.mapping.reference_map import build_reference_grid
            if os.path.exists(map_yaml) and os.path.exists(world):
                g, r, o = read_map(map_yaml)
                rg, rr, ro = build_reference_grid(World2D.from_sdf_file(world))
                with open(os.path.join(run_dir, 'map_quality.json'), 'w') as f:
                    json.dump(compare(g, r, o, rg, rr, ro), f, indent=2)
        except Exception as e:  # noqa: BLE001 - never fail during shutdown
            print(f'map comparison failed: {e}')
        try:
            from my_bot_autonomy.evaluation.report import make_report
            print('\n' + make_report(run_dir) + '\n')
        except Exception as e:  # noqa: BLE001 - never fail during shutdown
            print(f'report failed: {e}')

    actions = [
        LogInfo(msg=f'[full_mission_events] run folder: {run_dir}'),
        include('my_bot_bringup', 'robot.launch.py', {**{k: a[k] for k in ROBOT_ARGS}, 'run_dir': run_dir}),
        include('my_bot_bringup', 'stage_explore.launch.py', {
            **nav, 'slam_method': a['slam_method'], 'explorer': a['explorer'],
            'explore_strategy': a['explore_strategy'], 'run_dir': run_dir, 'shutdown_on_done': 'false'}),
        RegisterEventHandler(OnProcessExit(on_exit=on_explorer_exit)),
        RegisterEventHandler(OnShutdown(on_shutdown=on_shutdown)),
    ]
    if a['rviz'].lower() == 'true':
        actions.append(rviz_node('navigation.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription(declare(ROBOT_ARGS) + declare(NAV_ARGS) + declare(ARGS) +
                             [OpaqueFunction(function=launch_setup)])
