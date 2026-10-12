"""
PHASE 2 building block - start a simulator with the robot in it.

    ros2 launch my_bot_gazebo sim.launch.py                       # Gazebo + GUI
    ros2 launch my_bot_gazebo sim.launch.py gui:=false            # Gazebo headless (faster)
    ros2 launch my_bot_gazebo sim.launch.py sim:=lite             # tiny Python simulator, no GPU needed
    ros2 launch my_bot_gazebo sim.launch.py drive:=ros2_control   # ros2_control drive variant

This file is "hardware": after it runs you have exactly what a real robot driver
would give you:
    /scan  /imu  /wheel/odom_raw  /joint_states  /clock   and it listens on /cmd_vel
plus simulation-only extras: /ground_truth/odom and the /world/nav_room/* services.

It does NOT publish odom -> base_footprint TF; my_bot_bringup/robot.launch.py adds
the odometry pipeline on top of this file. Most of the time you start robot.launch.py
(or a phaseN launch file) rather than this one directly.

WHY OpaqueFunction?
-------------------
Launch arguments are "substitutions" that only get a value when the launch system
runs. OpaqueFunction lets us read them as plain Python strings (context.perform...)
so the logic below can use simple if/else - much easier to read for beginners.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_world(world_arg):
    """Accept either a file name inside my_bot_gazebo/worlds or an absolute path."""
    if os.path.isabs(world_arg):
        return world_arg
    return os.path.join(get_package_share_directory('my_bot_gazebo'), 'worlds', world_arg)


def launch_setup(context, *args, **kwargs):
    # ---- read every launch argument as a normal Python string ----
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    sim = arg('sim')
    gui = arg('gui').lower() == 'true'
    drive = arg('drive')
    world = resolve_world(arg('world'))
    x, y, yaw = arg('x'), arg('y'), arg('yaw')

    gazebo_pkg = get_package_share_directory('my_bot_gazebo')
    desc_pkg = get_package_share_directory('my_bot_description')
    actions = [LogInfo(msg=f'[sim.launch] sim={sim} drive={drive} world={world}')]

    # ---- 1. robot_state_publisher: URDF -> /robot_description + static TFs ----
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'drive': drive,
            'use_camera': arg('use_camera'),
            'lidar_noise': arg('lidar_noise'),
            # the lite simulator ignores <gazebo> tags, so we leave them out of its URDF
            'sim_mode': 'gazebo' if sim == 'gazebo' else 'lite',
        }.items()))

    if sim == 'gazebo':
        # ---- 2a. Gazebo Sim (Harmonic) ----
        # -r : start running immediately (not paused)
        # -s : server only (no GUI window) -> much lighter
        # -v 2 : verbosity (0..4). Raise to 4 when debugging plugin problems.
        gz_args = f'-r -v 2 {"" if gui else "-s "}{world}'
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()))

        # ---- 3a. Spawn the robot from the /robot_description topic ----
        spawn = Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-world', 'nav_room', '-topic', 'robot_description', '-name', 'my_bot',
                       '-x', x, '-y', y, '-z', '0.02', '-Y', yaw])
        actions.append(spawn)

        # ---- 4a. ROS <-> Gazebo bridge ----
        bridge_cfg = os.path.join(gazebo_pkg, 'config', f'bridge_{drive}.yaml')
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}]))

        # ---- 5a. ros2_control only: activate controllers once the robot exists ----
        if drive == 'ros2_control':
            jsb = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
            ddc = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'])
            # Event handler = "when the spawn process exits, THEN start the spawners".
            actions.append(RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb, ddc])))
            # Nav2/teleop publish geometry_msgs/Twist on /cmd_vel; Jazzy's diff_drive_controller
            # only accepts TwistStamped on /diff_drive_controller/cmd_vel. This tiny node converts.
            actions.append(Node(
                package='my_bot_autonomy', executable='twist_stamper', name='twist_stamper',
                output='screen',
                parameters=[{'use_sim_time': True, 'frame_id': 'base_footprint'}],
                remappings=[('cmd_vel_in', '/cmd_vel'),
                            ('cmd_vel_out', '/diff_drive_controller/cmd_vel')]))

    elif sim == 'lite':
        # ---- 2b. The lightweight Python simulator (2D kinematics + ray-cast LiDAR) ----
        # Publishes the SAME topics and offers the SAME /world/nav_room/* services as
        # Gazebo + bridge, so every other node works unchanged.
        actions.append(Node(
            package='my_bot_autonomy', executable='lite_sim', name='lite_sim', output='screen',
            parameters=[os.path.join(gazebo_pkg, 'config', 'lite_sim.yaml'), {
                'world_file': world,
                'spawn_x': float(x), 'spawn_y': float(y), 'spawn_yaw': float(yaw),
                'lidar_noise_std': float(arg('lidar_noise')),
                'use_sim_time': False,   # the simulator CREATES /clock, so it runs on wall time
            }]))
        if drive == 'ros2_control':
            actions.append(LogInfo(msg='[sim.launch] drive:=ros2_control is ignored with sim:=lite'))
    else:
        raise RuntimeError(f"Unknown sim '{sim}'. Use sim:=gazebo or sim:=lite")

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='gazebo', choices=['gazebo', 'lite'],
                              description='gazebo = Gazebo Sim Harmonic, lite = 2D Python simulator'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show the Gazebo window (false = headless server)'),
        DeclareLaunchArgument('world', default_value='nav_room.sdf',
                              description='World file name in my_bot_gazebo/worlds, or absolute path'),
        DeclareLaunchArgument('drive', default_value='gz_plugin', choices=['gz_plugin', 'ros2_control'],
                              description='Wheel drive implementation (Gazebo only)'),
        DeclareLaunchArgument('use_camera', default_value='false',
                              description='Simulate the camera (costs GPU/CPU)'),
        DeclareLaunchArgument('lidar_noise', default_value='0.01',
                              description='LiDAR range noise std-dev [m]'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn heading [rad]'),
        OpaqueFunction(function=launch_setup),
    ])
