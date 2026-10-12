"""
"Build your own Nav2" - the from-scratch learning stack on the simulated robot.

    ros2 launch my_bot_bringup learning_stack.launch.py
    ros2 launch my_bot_bringup learning_stack.launch.py algorithm:=dijkstra localizer:=amcl

  map_server (reference map)          -> /map
  learning_localizer (our particle filter, or AMCL)  -> TF map -> odom
  learning_planner (our A*/Dijkstra/greedy)          -> /learning/plan   (click "2D Goal Pose")
  learning_controller (our pure pursuit)             -> /cmd_vel
  learning_mapper (pose_source:=odom)                -> /learning/map  (see the odometry smear!)

The robot spawns at (0, 0) and the particle filter starts there. Click a goal in RViz
and watch every piece you wrote do its job. Then try the same goal with
phase5_navigation.launch.py and compare.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from my_bot_autonomy.launch_utils import ROBOT_ARGS, declare, include, resolve_run_dir, rviz_node, share, values


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ROBOT_ARGS) + ['run_dir', 'rviz', 'algorithm', 'localizer', 'map'])
    run_dir = resolve_run_dir(a['run_dir'])
    loc = 'learning_pf' if a['localizer'] == 'learning_pf' else 'amcl'
    actions = [
        include('my_bot_bringup', 'robot.launch.py', {**{k: a[k] for k in ROBOT_ARGS}, 'run_dir': run_dir}),
        include('my_bot_navigation', 'localization.launch.py', {
            'localization_method': loc, 'map': a['map'], 'set_initial_pose': 'true',
            'x': a['x'], 'y': a['y'], 'yaw': a['yaw']}),
        Node(package='my_bot_autonomy', executable='learning_planner', name='learning_planner', output='screen',
             parameters=[{'use_sim_time': True, 'algorithm': a['algorithm'], 'goal_topic': '/goal_pose'}]),
        Node(package='my_bot_autonomy', executable='learning_controller', name='learning_controller',
             output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='my_bot_autonomy', executable='learning_mapper', name='learning_mapper', output='screen',
             parameters=[{'use_sim_time': True, 'pose_source': 'odom'}]),
    ]
    if a['rviz'].lower() == 'true':
        actions.append(rviz_node('navigation.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription(declare(ROBOT_ARGS) + [
        DeclareLaunchArgument('algorithm', default_value='astar', description='astar | dijkstra | greedy'),
        DeclareLaunchArgument('localizer', default_value='learning_pf', description='learning_pf | amcl'),
        DeclareLaunchArgument('map', default_value=share('my_bot_navigation', 'maps', 'nav_room_reference.yaml')),
        DeclareLaunchArgument('run_dir', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup)])
