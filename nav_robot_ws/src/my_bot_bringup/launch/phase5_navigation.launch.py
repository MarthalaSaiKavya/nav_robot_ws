"""
PHASE 5 - Nav2 on a saved map: click goals in RViz ("2D Goal Pose").

    ros2 launch my_bot_bringup phase5_navigation.launch.py
    ros2 launch my_bot_bringup phase5_navigation.launch.py planner:=smac_2d controller:=mppi
    ros2 launch my_bot_bringup phase5_navigation.launch.py map:=$HOME/maps/my_room.yaml

Unknown-obstacle test while the robot drives (another terminal):
    ros2 run my_bot_autonomy sim_world spawn box1 1.5 -1.0
    ros2 run my_bot_autonomy sim_world remove box1
Headless goal (no RViz):
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: -4.2, y: -4.2}, orientation: {w: 1.0}}}}" --feedback
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from my_bot_autonomy.launch_utils import NAV_ARGS, ROBOT_ARGS, declare, include, resolve_run_dir, rviz_node, share, values

LOC_ARGS = {
    'localization_method': ('amcl', 'amcl | slam_toolbox | learning_pf'),
    'map': (share('my_bot_navigation', 'maps', 'nav_room_reference.yaml'), 'Map yaml'),
    'amcl_profile': ('default', 'AMCL profile'),
}


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ROBOT_ARGS) + list(LOC_ARGS) + list(NAV_ARGS) + ['run_dir', 'rviz'])
    run_dir = resolve_run_dir(a['run_dir'])
    actions = [include('my_bot_bringup', 'robot.launch.py', {**{k: a[k] for k in ROBOT_ARGS}, 'run_dir': run_dir}),
               # the robot is spawned at (x, y, yaw) and the localizer is told exactly that
               include('my_bot_navigation', 'localization.launch.py', {
                   'localization_method': a['localization_method'], 'map': a['map'], 'amcl_profile': a['amcl_profile'],
                   'set_initial_pose': 'true', 'x': a['x'], 'y': a['y'], 'yaw': a['yaw']}),
               include('my_bot_navigation', 'navigation.launch.py', {k: a[k] for k in NAV_ARGS})]
    if a['rviz'].lower() == 'true':
        actions.append(rviz_node('navigation.rviz'))
    return actions


def generate_launch_description():
    return LaunchDescription(declare(ROBOT_ARGS) + declare(LOC_ARGS) + declare(NAV_ARGS) + [
        DeclareLaunchArgument('run_dir', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup)])
