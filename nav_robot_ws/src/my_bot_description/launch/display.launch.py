"""
PHASE 1 - Look at the robot model and its TF tree in RViz.  No simulator.

    ros2 launch my_bot_description display.launch.py
    ros2 launch my_bot_description display.launch.py gui:=false     # no slider window

What you should see:
  * RViz with the blue robot, red LiDAR, white camera.
  * A "Joint State Publisher" window with two sliders (left/right wheel).
    Moving a slider spins that wheel in RViz -> proves the wheel joints work.
  * The TF display shows every frame. Turn on "Show Names".

Verification commands (second terminal):
  ros2 run tf2_tools view_frames            # writes frames.pdf with the TF tree
  ros2 run tf2_ros tf2_echo base_link lidar_link    # expect  [0.050, 0.000, 0.105]
  ros2 run tf2_ros tf2_echo base_link camera_optical_link
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('my_bot_description')
    gui = LaunchConfiguration('gui')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg, 'launch', 'rsp.launch.py'])),
        launch_arguments={
            'use_sim_time': 'false',      # no simulator -> use the wall clock
            'sim_mode': 'none',           # skip all <gazebo> plugins
            'use_camera': 'false',
        }.items(),
    )

    # Without a simulator nobody publishes /joint_states, so the wheel TFs would be
    # missing. joint_state_publisher(_gui) fills that gap with slider values.
    jsp_gui = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
                   condition=IfCondition(gui))
    jsp = Node(package='joint_state_publisher', executable='joint_state_publisher',
               condition=UnlessCondition(gui))

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'display.rviz'])],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show joint slider window'),
        rsp, jsp_gui, jsp, rviz,
    ])
