"""
robot_state_publisher (RSP) launch - reused by every other launch file.

WHAT RSP DOES
-------------
1. Reads the URDF (we generate it from xacro right here).
2. Publishes it as a string on the /robot_description topic (Gazebo spawns the robot
   from that topic; RViz draws it from that topic).
3. Publishes TF for every FIXED joint once on /tf_static
   (base_footprint->base_link, base_link->lidar_link, lidar_link->camera_link ...).
4. Listens to /joint_states and publishes TF for MOVING joints (the wheels) on /tf.

It does NOT publish odom->base_footprint. That transform comes from odometry
(see my_bot_autonomy/robot/odometry_processor.py).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    drive = LaunchConfiguration('drive')
    use_camera = LaunchConfiguration('use_camera')
    sim_mode = LaunchConfiguration('sim_mode')
    lidar_noise = LaunchConfiguration('lidar_noise')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('my_bot_description'), 'urdf', 'my_bot.urdf.xacro'])

    # Command([...]) runs "xacro <file> drive:=... use_camera:=..." when the launch
    # file starts and captures the printed URDF as a string.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' drive:=', drive,
                 ' use_camera:=', use_camera,
                 ' sim_mode:=', sim_mode,
                 ' lidar_noise:=', lidar_noise]),
        value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the /clock topic from the simulator'),
        DeclareLaunchArgument('drive', default_value='gz_plugin',
                              choices=['gz_plugin', 'ros2_control'],
                              description='How Gazebo drives the wheels'),
        DeclareLaunchArgument('use_camera', default_value='false',
                              description='Add the simulated camera sensor'),
        DeclareLaunchArgument('sim_mode', default_value='gazebo',
                              choices=['gazebo', 'lite', 'none'],
                              description='Which simulator the URDF is generated for'),
        DeclareLaunchArgument('lidar_noise', default_value='0.01',
                              description='LiDAR range noise std-dev [m]'),
        rsp,
    ])
