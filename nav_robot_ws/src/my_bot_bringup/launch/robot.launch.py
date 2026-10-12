"""
"The robot is alive": simulator + robot + odometry pipeline (+ optional EKF, evaluator).

Every phase launch file includes this one. After it runs you have, exactly like a real
robot with a driver:
    /scan /imu /odom /joint_states /clock     TF: odom -> base_footprint -> base_link -> sensors
and it obeys /cmd_vel.

    ros2 launch my_bot_bringup robot.launch.py                          # Gazebo, realistic odometry
    ros2 launch my_bot_bringup robot.launch.py sim:=lite                # no Gazebo needed
    ros2 launch my_bot_bringup robot.launch.py odometry_profile:=terrible use_ekf:=true

ODOMETRY PIPELINE (who publishes odom -> base_footprint?):

    simulator --/wheel/odom_raw--> odometry_processor --/odom + TF-->              (use_ekf:=false)
    simulator --/wheel/odom_raw--> odometry_processor --/wheel/odom--> EKF --/odom + TF-->  (use_ekf:=true)
                                   (adds the drift of odometry_profile)   (+ /imu)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node

from my_bot_autonomy.launch_utils import ROBOT_ARGS, declare, include, resolve_run_dir, share, values


def launch_setup(context, *args, **kwargs):
    a = values(context, list(ROBOT_ARGS) + ['run_dir'])
    use_ekf = a['use_ekf'].lower() == 'true'
    ros2_control = a['drive'] == 'ros2_control' and a['sim'] == 'gazebo'
    actions = [include('my_bot_gazebo', 'sim.launch.py',
                       {k: a[k] for k in ('sim', 'gui', 'world', 'drive', 'use_camera', 'lidar_noise', 'x', 'y', 'yaw')})]

    actions.append(Node(
        package='my_bot_autonomy', executable='odometry_processor', name='odometry_processor', output='screen',
        parameters=[share('my_bot_bringup', 'config', 'odometry', f'{a["odometry_profile"]}.yaml'), {
            'use_sim_time': True,
            'input_topic': '/diff_drive_controller/odom' if ros2_control else '/wheel/odom_raw',
            'output_topic': '/wheel/odom' if use_ekf else '/odom',
            'publish_tf': not use_ekf,
        }]))

    if use_ekf:
        actions.append(Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
            parameters=[share('my_bot_navigation', 'config', 'odometry', 'ekf.yaml'), {'use_sim_time': True}],
            remappings=[('odometry/filtered', '/odom')]))

    if a['use_evaluator'].lower() == 'true':
        run_dir = resolve_run_dir(a['run_dir'])
        actions.append(LogInfo(msg=f'[robot.launch] run folder: {run_dir}'))
        actions.append(Node(package='my_bot_autonomy', executable='evaluator', name='evaluator', output='screen',
                            parameters=[{'use_sim_time': True, 'run_dir': run_dir}]))
    return actions


def generate_launch_description():
    return LaunchDescription(declare(ROBOT_ARGS) + [
        DeclareLaunchArgument('run_dir', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
