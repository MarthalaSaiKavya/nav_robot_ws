"""
PHASE 3 - SLAM: build a map while localizing in it.

    ros2 launch my_bot_navigation slam.launch.py                               # slam_toolbox async
    ros2 launch my_bot_navigation slam.launch.py slam_method:=slam_toolbox_sync
    ros2 launch my_bot_navigation slam.launch.py slam_method:=cartographer     # needs ros-jazzy-cartographer-ros
    ros2 launch my_bot_navigation slam.launch.py slam_method:=learning_mapper  # our log-odds mapper + perfect poses

Usually started for you by my_bot_bringup/phase3_slam.launch.py (sim + teleop instructions)
or stage_explore.launch.py (autonomous).

Every variant provides the SAME interface to the rest of the system:
    /map  (nav_msgs/OccupancyGrid, transient local)      and      TF  map -> odom
plus nav2's map_saver_server (/map_saver/save_map) so maps can be saved the same way.

LIFECYCLE NODES: slam_toolbox (Jazzy) and nav2's map_saver are "managed" nodes: they start
UNCONFIGURED and must be told configure -> activate. Two ways are shown here:
  * map_saver    -> nav2_lifecycle_manager (also keeps a "bond" heartbeat with it)
  * slam_toolbox -> launch EVENTS (slam_toolbox_lifecycle_node below), like upstream does
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from my_bot_autonomy.launch_utils import slam_toolbox_lifecycle_node


def launch_setup(context, *args, **kwargs):
    method = LaunchConfiguration('slam_method').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    pkg = get_package_share_directory('my_bot_navigation')
    nav2_cfg = os.path.join(pkg, 'config', 'nav2', 'nav2_base.yaml')
    common = {'use_sim_time': use_sim_time}
    actions = [LogInfo(msg=f'[slam.launch] slam_method={method}')]
    managed = ['map_saver']

    # nav2 map saver server: offers /map_saver/save_map for every SLAM variant
    actions.append(Node(package='nav2_map_server', executable='map_saver_server', name='map_saver',
                        output='screen', parameters=[nav2_cfg, common]))

    if method in ('slam_toolbox_async', 'slam_toolbox_sync'):
        flavour = method.split('_')[-1]
        actions += slam_toolbox_lifecycle_node(
            f'{flavour}_slam_toolbox_node',
            [os.path.join(pkg, 'config', 'slam', f'slam_toolbox_{flavour}.yaml'), common])

    elif method == 'cartographer':
        cfg_dir = os.path.join(pkg, 'config', 'slam')
        actions.append(Node(
            package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
            output='screen', parameters=[common],
            arguments=['-configuration_directory', cfg_dir, '-configuration_basename', 'cartographer_2d.lua']))
        # cartographer builds SUBMAPS; this second node renders them into one /map
        actions.append(Node(
            package='cartographer_ros', executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node', output='screen', parameters=[common],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']))

    elif method == 'learning_mapper':
        actions.append(Node(
            package='my_bot_autonomy', executable='learning_mapper', name='learning_mapper', output='screen',
            parameters=[{'pose_source': 'ground_truth', 'publish_as_slam': True}, common]))
    else:
        raise RuntimeError(f'unknown slam_method {method}')

    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_slam',
        output='screen', parameters=[{'autostart': True, 'node_names': managed}, common]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('slam_method', default_value='slam_toolbox_async',
                              choices=['slam_toolbox_async', 'slam_toolbox_sync', 'cartographer', 'learning_mapper'],
                              description='Which SLAM implementation to run'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
