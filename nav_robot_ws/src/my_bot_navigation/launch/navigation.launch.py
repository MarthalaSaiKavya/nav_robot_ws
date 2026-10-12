"""
PHASE 5 - Nav2: plan, follow, recover.   (needs a map + map->odom from SLAM or localization)

    ros2 launch my_bot_navigation navigation.launch.py
    ros2 launch my_bot_navigation navigation.launch.py planner:=smac_2d controller:=mppi
    ros2 launch my_bot_navigation navigation.launch.py bt:=patient use_collision_monitor:=false

  planner     navfn_dijkstra | navfn_astar | smac_2d | theta_star | smac_hybrid | smac_lattice
  controller  rpp | dwb | mppi | graceful | rotation_shim
  bt          default | nav2_jazzy_default | patient | no_recovery | replan_on_invalid  (behavior_trees/*.xml)

We start every Nav2 server EXPLICITLY here (instead of including nav2_bringup) so that
you can see the whole system in one file. It mirrors nav2_bringup/navigation_launch.py
(Jazzy) minus the docking and route servers.

Velocity command chain (remappings below make it happen):
   controller_server / behavior_server --cmd_vel_nav--> velocity_smoother
     --cmd_vel_smoothed--> collision_monitor --/cmd_vel--> robot
   (use_collision_monitor:=false -> velocity_smoother publishes /cmd_vel directly)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

PLANNERS = ['navfn_dijkstra', 'navfn_astar', 'smac_2d', 'theta_star', 'smac_hybrid', 'smac_lattice']
CONTROLLERS = ['rpp', 'dwb', 'mppi', 'graceful', 'rotation_shim']
BTS = ['default', 'nav2_jazzy_default', 'patient', 'no_recovery', 'replan_on_invalid']


def launch_setup(context, *args, **kwargs):
    def arg(n):
        return LaunchConfiguration(n).perform(context)
    planner, controller, bt = arg('planner'), arg('controller'), arg('bt')
    use_sim_time = arg('use_sim_time').lower() == 'true'
    use_cm = arg('use_collision_monitor').lower() == 'true'
    log_level = arg('log_level')
    pkg = get_package_share_directory('my_bot_navigation')
    cfg = os.path.join(pkg, 'config', 'nav2')

    # allow_substs=True expands $(find-pkg-share ...) inside the yaml (used by smac_lattice)
    params = [ParameterFile(os.path.join(cfg, 'nav2_base.yaml'), allow_substs=True),
              ParameterFile(os.path.join(cfg, 'planners', f'{planner}.yaml'), allow_substs=True),
              ParameterFile(os.path.join(cfg, 'controllers', f'{controller}.yaml'), allow_substs=True)]
    extra = arg('extra_params_file')
    if extra:
        params.append(ParameterFile(os.path.expanduser(extra), allow_substs=True))
    params.append({'use_sim_time': use_sim_time, 'autostart': True})

    bt_xml = os.path.join(pkg, 'behavior_trees', f'navigate_to_pose_{bt}.xml')
    bt_params = params + [{'default_nav_to_pose_bt_xml': bt_xml}]
    ros_args = ['--ros-args', '--log-level', log_level]
    nav_cmd = [('cmd_vel', 'cmd_vel_nav')]

    def server(pkg_name, exe, name, extra_params=None, remaps=()):
        return Node(package=pkg_name, executable=exe, name=name, output='screen',
                    parameters=extra_params or params, arguments=ros_args, remappings=list(remaps))

    nodes = [
        LogInfo(msg=f'[navigation.launch] planner={planner} controller={controller} bt={bt} '
                    f'collision_monitor={use_cm}'),
        server('nav2_controller', 'controller_server', 'controller_server', remaps=nav_cmd),
        server('nav2_smoother', 'smoother_server', 'smoother_server'),
        server('nav2_planner', 'planner_server', 'planner_server'),
        server('nav2_behaviors', 'behavior_server', 'behavior_server', remaps=nav_cmd),
        server('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', extra_params=bt_params),
        server('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        server('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
               remaps=nav_cmd + ([] if use_cm else [('cmd_vel_smoothed', 'cmd_vel')])),
    ]
    managed = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
               'velocity_smoother', 'bt_navigator', 'waypoint_follower']
    if use_cm:
        nodes.append(server('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'))
        managed.insert(5, 'collision_monitor')

    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
        output='screen', arguments=ros_args,
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': managed}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('planner', default_value='navfn_dijkstra', choices=PLANNERS),
        DeclareLaunchArgument('controller', default_value='rpp', choices=CONTROLLERS),
        DeclareLaunchArgument('bt', default_value='default', choices=BTS),
        DeclareLaunchArgument('use_collision_monitor', default_value='true'),
        DeclareLaunchArgument('extra_params_file', default_value='',
                              description='Optional yaml that overrides any Nav2 parameter (applied last)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=launch_setup),
    ])
