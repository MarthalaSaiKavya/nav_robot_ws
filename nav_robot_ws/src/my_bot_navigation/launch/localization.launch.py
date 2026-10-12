"""
PHASE 4 - Localization on a SAVED map (no more mapping).

    ros2 launch my_bot_navigation localization.launch.py map:=/path/to/map.yaml
    ros2 launch my_bot_navigation localization.launch.py localization_method:=slam_toolbox map:=/path/to/map.yaml
    ros2 launch my_bot_navigation localization.launch.py localization_method:=learning_pf
    ros2 launch my_bot_navigation localization.launch.py amcl_profile:=beam_model

  localization_method
     amcl          nav2_amcl particle filter on map_server's map           (default)
     slam_toolbox  slam_toolbox in localization mode on the saved POSE GRAPH
                   (needs <map>.posegraph/.data next to <map>.yaml - the explorer writes them)
     learning_pf   our own particle filter (my_bot_autonomy/learning/particle_filter.py)

  Initial pose: set_initial_pose:=true x:=.. y:=.. yaw:=..  -> given to the localizer at start.
  Otherwise use RViz "2D Pose Estimate" or let mission_runner publish /initialpose.

Interface for the rest of the system: TF map -> odom, and /map (transient local).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from my_bot_autonomy.launch_utils import slam_toolbox_lifecycle_node


def launch_setup(context, *args, **kwargs):
    def arg(n):
        return LaunchConfiguration(n).perform(context)
    method = arg('localization_method')
    map_yaml = os.path.expanduser(arg('map'))
    use_sim_time = arg('use_sim_time').lower() == 'true'
    set_init = arg('set_initial_pose').lower() == 'true'
    x, y, yaw = float(arg('x')), float(arg('y')), float(arg('yaw'))
    pkg = get_package_share_directory('my_bot_navigation')
    common = {'use_sim_time': use_sim_time}
    actions = [LogInfo(msg=f'[localization.launch] method={method} map={map_yaml} '
                           f'initial_pose={"(%.2f, %.2f, %.2f)" % (x, y, yaw) if set_init else "from /initialpose"}')]
    if not os.path.exists(map_yaml):
        raise RuntimeError(f'map file not found: {map_yaml}')
    managed = []

    if method in ('amcl', 'learning_pf'):
        # map_server: loads the PGM+YAML and publishes /map once (latched / transient local)
        actions.append(Node(package='nav2_map_server', executable='map_server', name='map_server',
                            output='screen',
                            parameters=[os.path.join(pkg, 'config', 'nav2', 'nav2_base.yaml'),
                                        {'yaml_filename': map_yaml}, common]))
        managed.append('map_server')

    if method == 'amcl':
        profile = os.path.join(pkg, 'config', 'localization', 'amcl_profiles', f'{arg("amcl_profile")}.yaml')
        actions.append(Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[os.path.join(pkg, 'config', 'localization', 'amcl.yaml'), profile, {
                'set_initial_pose': set_init,
                'initial_pose.x': x, 'initial_pose.y': y, 'initial_pose.z': 0.0, 'initial_pose.yaw': yaw,
            }, common]))
        managed.append('amcl')

    elif method == 'slam_toolbox':
        base = os.path.splitext(map_yaml)[0]
        if not os.path.exists(base + '.posegraph'):
            raise RuntimeError(f'{base}.posegraph not found - slam_toolbox localization needs the serialized '
                               'pose graph (saved by the explorer / `ros2 run my_bot_autonomy save_map`).')
        actions += slam_toolbox_lifecycle_node('localization_slam_toolbox_node', [
            os.path.join(pkg, 'config', 'slam', 'slam_toolbox_localization.yaml'),
            {'map_file_name': base, 'map_start_pose': [x, y, yaw]}, common])

    elif method == 'learning_pf':
        actions.append(Node(
            package='my_bot_autonomy', executable='learning_localizer', name='learning_localizer', output='screen',
            parameters=[{'publish_tf': True, 'set_initial_pose': set_init,
                         'initial_pose.x': x, 'initial_pose.y': y, 'initial_pose.yaw': yaw}, common]))
    else:
        raise RuntimeError(f'unknown localization_method {method}')

    if managed:
        actions.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
            output='screen', parameters=[{'autostart': True, 'node_names': managed}, common]))
    return actions


def generate_launch_description():
    default_map = os.path.join(get_package_share_directory('my_bot_navigation'), 'maps', 'nav_room_reference.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('localization_method', default_value='amcl',
                              choices=['amcl', 'slam_toolbox', 'learning_pf']),
        DeclareLaunchArgument('map', default_value=default_map, description='Map yaml to localize in'),
        DeclareLaunchArgument('amcl_profile', default_value='default',
                              choices=['default', 'beam_model', 'many_particles', 'kidnap_recovery', 'fast']),
        DeclareLaunchArgument('set_initial_pose', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
