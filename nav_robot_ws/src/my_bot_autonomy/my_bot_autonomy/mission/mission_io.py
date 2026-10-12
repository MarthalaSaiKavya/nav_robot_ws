"""
Loading + validating mission and scenario YAML files (pure Python, no ROS).

MISSION FILE (my_bot_bringup/config/missions/*.yaml):

    mission:
      name: demo_mission
      frame: map
      start_pose: {x: 1.0, y: -4.2, yaw: 3.14}           # where the robot REALLY is
      initial_guess_error: {x: 0.35, y: -0.25, yaw: 0.25} # AMCL guess = start + error
      goal_timeout: 150.0          # seconds per attempt (can be overridden per goal)
      retries_per_goal: 1          # immediate retries after a failure
      retry_failed_at_end: true    # one more try for every failed goal at the end
      goals:
        - {name: corner, x: -4.2, y: -4.2, yaw: 0.0}
        - {name: east,   x:  4.2, y: -0.5, yaw: 1.57, timeout: 120}

SCENARIO FILE (my_bot_bringup/config/scenarios/*.yaml): see scenario_director.py.
"""
import math
import os

import yaml


def _pose(d, default=(0.0, 0.0, 0.0)):
    if d is None:
        return default
    return (float(d.get('x', 0.0)), float(d.get('y', 0.0)), float(d.get('yaw', 0.0)))


def resolve_config(path, subdir):
    """Accepts an absolute path or a file name inside my_bot_bringup/config/<subdir>/."""
    path = os.path.expanduser(path)
    if os.path.isfile(path):
        return path
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(get_package_share_directory('my_bot_bringup'), 'config', subdir, path)
        if os.path.isfile(cand):
            return cand
    except Exception:
        pass
    raise FileNotFoundError(f'{subdir} file not found: {path}')


def load_mission(path):
    with open(resolve_config(path, 'missions')) as f:
        data = yaml.safe_load(f)['mission']
    goals = []
    for i, g in enumerate(data.get('goals', [])):
        goals.append({'index': i, 'name': g.get('name', f'goal_{i}'),
                      'x': float(g['x']), 'y': float(g['y']), 'yaw': float(g.get('yaw', 0.0)),
                      'timeout': float(g.get('timeout', data.get('goal_timeout', 150.0)))})
    start = _pose(data.get('start_pose'))
    err = _pose(data.get('initial_guess_error'))
    return {
        'name': data.get('name', os.path.splitext(os.path.basename(path))[0]),
        'frame': data.get('frame', 'map'),
        'start_pose': start,
        'initial_guess_error': err,
        'initial_guess': (start[0] + err[0], start[1] + err[1], start[2] + err[2]),
        'goal_timeout': float(data.get('goal_timeout', 150.0)),
        'retries_per_goal': int(data.get('retries_per_goal', 1)),
        'retry_failed_at_end': bool(data.get('retry_failed_at_end', True)),
        'goals': goals,
    }


def load_scenario(path):
    if not path or path in ('none', 'None'):
        return {'name': 'none', 'events': []}
    with open(resolve_config(path, 'scenarios')) as f:
        data = yaml.safe_load(f)['scenario']
    events = []
    for i, e in enumerate(data.get('events', [])):
        events.append({'name': e.get('name', f'event_{i}'), 'trigger': e['trigger'], 'action': e['action'],
                       'once': bool(e.get('once', True)), 'fired': 0})
    return {'name': data.get('name', 'scenario'), 'events': events}


def validate_goals(mission, world, robot_radius=0.24, margin=0.15):
    """Returns a list of human-readable problems (empty = all goals are reachable spots).
    `world` is a common.world_model.World2D."""
    problems = []
    poses = [('start_pose', *mission['start_pose'])] + [(g['name'], g['x'], g['y'], g['yaw'])
                                                      for g in mission['goals']]
    for name, x, y, _ in poses:
        d = world.distance_to_obstacles(x, y)
        if d < robot_radius + margin:
            problems.append(f'{name} ({x:.2f}, {y:.2f}) is only {d:.2f} m from an obstacle '
                            f'(need >= {robot_radius + margin:.2f})')
        if not (math.isfinite(x) and math.isfinite(y)):
            problems.append(f'{name} has invalid coordinates')
    return problems
