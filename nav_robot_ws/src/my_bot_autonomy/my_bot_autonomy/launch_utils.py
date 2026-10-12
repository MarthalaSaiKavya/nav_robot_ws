"""
Helpers shared by the launch files in my_bot_bringup (kept here so they are importable
as a normal Python module: `from my_bot_autonomy.launch_utils import ...`).

ALL LAUNCH ARGUMENTS OF THE PROJECT, grouped (every phase launch file accepts the ones it uses):

  SIMULATION / ROBOT      sim:=gazebo|lite   gui:=true|false   world:=nav_room.sdf|nav_room_simple.sdf
                          drive:=gz_plugin|ros2_control   use_camera:=false   lidar_noise:=0.01
                          x:= y:= yaw:=  (spawn pose)
  ODOMETRY                odometry_profile:=perfect|realistic|terrible   use_ekf:=false|true
  SLAM                    slam_method:=slam_toolbox_async|slam_toolbox_sync|cartographer|learning_mapper
  LOCALIZATION            localization_method:=amcl|slam_toolbox|learning_pf
                          amcl_profile:=default|beam_model|many_particles|kidnap_recovery|fast   map:=<yaml>
  NAV2                    planner:=navfn_dijkstra|navfn_astar|smac_2d|theta_star|smac_hybrid|smac_lattice
                          controller:=rpp|dwb|mppi|graceful|rotation_shim
                          bt:=default|nav2_jazzy_default|patient|no_recovery|replan_on_invalid
                          use_collision_monitor:=true
  EXPLORATION             explorer:=frontier|explore_lite|reactive
                          explore_strategy:=nearest|largest|utility|information_gain|random
                          reactive_mode:=wall_follow|random_walk   use_path_distance:=false
  MISSION                 mission_file:=demo_mission.yaml   mission_method:=navigate_to_pose|follow_waypoints|
                          through_poses|raw_action   initial_pose_mode:=topic|params|global|none
                          scenario_file:=demo_scenario.yaml|surprise_only.yaml|hard_block_only.yaml|none
  OUTPUT                  run_dir:=  (default ~/nav_robot_runs/<timestamp>)   rviz:=true
"""
import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROBOT_ARGS = {
    'sim': ('gazebo', 'gazebo = Gazebo Sim (Harmonic), lite = lightweight Python simulator'),
    'gui': ('true', 'Show the Gazebo window'),
    'world': ('nav_room.sdf', 'World file in my_bot_gazebo/worlds or an absolute path'),
    'drive': ('gz_plugin', 'gz_plugin | ros2_control'),
    'use_camera': ('false', 'Simulate the camera sensor'),
    'lidar_noise': ('0.01', 'LiDAR noise std-dev [m]'),
    'x': ('0.0', 'Spawn x [m]'),
    'y': ('0.0', 'Spawn y [m]'),
    'yaw': ('0.0', 'Spawn yaw [rad]'),
    'odometry_profile': ('realistic', 'perfect | realistic | terrible (simulated wheel-odometry errors)'),
    'use_ekf': ('false', 'Fuse wheel odometry + IMU with robot_localization'),
    'use_evaluator': ('true', 'Log localization error vs ground truth'),
}
NAV_ARGS = {
    'planner': ('navfn_dijkstra', 'navfn_dijkstra | navfn_astar | smac_2d | theta_star | smac_hybrid | smac_lattice'),
    'controller': ('rpp', 'rpp | dwb | mppi | graceful | rotation_shim'),
    'bt': ('default', 'default | nav2_jazzy_default | patient | no_recovery | replan_on_invalid'),
    'use_collision_monitor': ('true', 'Insert Nav2 collision_monitor before /cmd_vel'),
}


def share(pkg, *parts):
    return os.path.join(get_package_share_directory(pkg), *parts)


def declare(args):
    """dict {name: (default, description)} -> list of DeclareLaunchArgument."""
    return [DeclareLaunchArgument(k, default_value=v[0], description=v[1]) for k, v in args.items()]


def values(context, names):
    """Read launch arguments as plain strings."""
    return {n: LaunchConfiguration(n).perform(context) for n in names}


def include(pkg, launch_file, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share(pkg, 'launch', launch_file)),
        launch_arguments=list((args or {}).items()))


def resolve_run_dir(run_dir):
    """'' -> ~/nav_robot_runs/<timestamp> (created). Every node of one run writes there."""
    if not run_dir:
        root = os.path.expanduser(os.environ.get('MY_BOT_RUNS', '~/nav_robot_runs'))
        run_dir = os.path.join(root, datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    run_dir = os.path.expanduser(run_dir)
    os.makedirs(run_dir, exist_ok=True)
    return run_dir


def rviz_node(config='navigation.rviz'):
    from launch_ros.actions import Node
    return Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                arguments=['-d', share('my_bot_bringup', 'rviz', config)],
                parameters=[{'use_sim_time': True}])


def slam_toolbox_lifecycle_node(executable, params):
    """slam_toolbox (Jazzy) is a LIFECYCLE node. Like slam_toolbox's own launch files (and Nav2's
    slam_launch.py) we drive it through configure -> activate with launch EVENTS:
      1. EmitEvent(ChangeState CONFIGURE) as soon as the process starts
      2. when it reaches 'inactive', EmitEvent(ChangeState ACTIVATE)
    (Alternative: use_lifecycle_manager:=true and add it to a nav2 lifecycle_manager's node list.)"""
    from launch.actions import EmitEvent, RegisterEventHandler
    from launch.events import matches_action
    from launch_ros.actions import LifecycleNode
    from launch_ros.event_handlers import OnStateTransition
    from launch_ros.events.lifecycle import ChangeState
    from lifecycle_msgs.msg import Transition
    node = LifecycleNode(package='slam_toolbox', executable=executable, name='slam_toolbox', namespace='',
                         output='screen', parameters=params + [{'use_lifecycle_manager': False}])
    configure = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                            transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                              transition_id=Transition.TRANSITION_ACTIVATE))]))
    return [node, configure, activate]
