# Architecture

## The big picture

```
          ┌──────────────────────── simulator (Gazebo + ros_gz_bridge  |  lite_sim) ────────────────────────┐
          │  /scan  /imu  /wheel/odom_raw  /joint_states  /clock      /ground_truth/odom (evaluation only)    │
          │  listens: /cmd_vel           services: /world/nav_room/create | remove | set_pose               │
          └───────┬───────────────────────────────┬────────────────────────────────────────▲─────────────────┘
                  │                               │                                        │ /cmd_vel
   robot_state_publisher               odometry_processor (+ EKF)                 collision_monitor
   URDF -> static TF + wheel TF        /odom  + TF odom->base_footprint                  ▲ cmd_vel_smoothed
                  │                               │                               velocity_smoother
                  ▼                               ▼                                        ▲ cmd_vel_nav
        ┌─────────────────────── SLAM (stage 1)  or  LOCALIZATION (stage 2) ───┐   controller_server / behavior_server
        │ slam_toolbox | cartographer | learning_mapper                      │           ▲ FollowPath / Spin / BackUp / Wait
        │ map_server + amcl | slam_toolbox (loc.) | learning_pf              │   bt_navigator  (behaviour tree)
        │ -> /map  + TF map->odom                                            │           ▲ ComputePathToPose
        └───────────────────────────────┬─────────────────────────────────────┘   planner_server (global_costmap)
                                        │                                                  ▲
                                        └──────────────────────► costmaps ◄────────────────┘
                                                                     ▲
                         goals: frontier_explorer (stage 1)  |  mission_runner (stage 2)  |  RViz "2D Goal Pose"
                         world changes: scenario_director  ── /world/nav_room/* ──► simulator
                         measurements: evaluator (/ground_truth/odom vs TF)          orchestration: supervisor
```

## TF tree (REP-105)

```
map ──(SLAM / AMCL / PF)──► odom ──(odometry_processor or EKF)──► base_footprint
                                                                   └─► base_link (robot_state_publisher)
                                                                         ├─ chassis_link
                                                                         ├─ left_wheel_link, right_wheel_link  (moving: /joint_states)
                                                                         ├─ caster_front_link, caster_rear_link
                                                                         ├─ imu_link
                                                                         └─ lidar_link ─► camera_link ─► camera_optical_link
```

*Why map → odom and not map → base?* Every frame has exactly one parent. Odometry is smooth but
drifts; SLAM/AMCL are drift-free but jump. Publishing the *correction* `map → odom` keeps both
properties: controllers use smooth `odom`, planners use drift-free `map`.

The simulator's `world` frame is never in TF; in this project it coincides with `map` because
exploration starts at the world origin (0, 0, 0). That lets the evaluator compare poses directly.

## Topics you will look at most

| Topic | Type | From → To |
|---|---|---|
| `/scan` | sensor_msgs/LaserScan | simulator → SLAM, AMCL, costmaps, collision_monitor, evaluator |
| `/odom` | nav_msgs/Odometry | odometry_processor or EKF → Nav2 (velocity smoother, controllers) |
| `/map` | nav_msgs/OccupancyGrid (transient local) | SLAM / map_server → costmaps, explorer, AMCL |
| `/plan`, `/local_plan` | nav_msgs/Path | planner / controller → RViz, scenario_director |
| `/global_costmap/costmap`, `/local_costmap/costmap` | OccupancyGrid | costmaps → RViz |
| `/particle_cloud` | nav2_msgs/ParticleCloud | AMCL → RViz |
| `/cmd_vel_nav` → `/cmd_vel_smoothed` → `/cmd_vel` | geometry_msgs/Twist | the velocity chain (see below) |
| `/initialpose` | PoseWithCovarianceStamped | RViz / mission_runner → AMCL / PF |
| `/goal_pose` | PoseStamped | RViz → bt_navigator |
| `/mission/events`, `/exploration/status` | std_msgs/String (JSON) | mission_runner / explorer → scenario_director, you |
| `/evaluation/localization_error` | std_msgs/Float32 | evaluator → you |

## The velocity command chain

```
controller_server ─┐
                   ├─ cmd_vel_nav ─► velocity_smoother ─ cmd_vel_smoothed ─► collision_monitor ─ /cmd_vel ─► robot
behavior_server ───┘                 (acceleration limits)                    (last-resort safety)
```
`use_collision_monitor:=false` remaps the smoother output straight to `/cmd_vel`.
With `drive:=ros2_control`: `/cmd_vel` ─► twist_stamper ─► `/diff_drive_controller/cmd_vel` (TwistStamped).

## Lifecycle (managed) nodes

Nav2 servers, AMCL, map_server and map_saver start **unconfigured**; `nav2_lifecycle_manager`
calls configure → activate on the list of nodes it is given, in order, and keeps a bond
(heartbeat) to restart/stop the group if one dies. Three managers are used:
`lifecycle_manager_slam` (map_saver), `lifecycle_manager_localization` (map_server, amcl),
`lifecycle_manager_navigation` (all Nav2 servers). slam_toolbox is activated with launch events.

Inspect: `ros2 lifecycle get /amcl`, `ros2 lifecycle nodes`.

**Gotcha:** an action server *exists* as soon as its node starts, but only an *active* node
accepts goals. Anything that sends goals must wait for `active` first: `mission_runner` and
`frontier_explorer` call `/bt_navigator/get_state`, and the `wait_for_nav2` gate process
does the same for explore_lite (started by an `OnProcessExit` event when the gate exits).

## Launch-file structure

```
full_mission.launch.py ── robot.launch.py ── my_bot_gazebo/sim.launch.py ── my_bot_description/rsp.launch.py
     │                         └ odometry_processor, EKF, evaluator
     └ supervisor ─(subprocess)─► stage_explore.launch.py ── slam.launch.py + navigation.launch.py + explorer
                  ─(subprocess)─► stage_mission.launch.py ── localization.launch.py + navigation.launch.py
                                                              + mission_runner + scenario_director
phaseN_*.launch.py = robot.launch.py + the pieces of that phase (+ RViz)
```
Shared argument lists and helpers: `my_bot_autonomy/launch_utils.py`.
Most launch files use `OpaqueFunction` so the logic can be plain Python `if/else` on argument values.

## Threading pattern of the Python nodes

Long sequential logic (a mission, an exploration loop) runs in a **worker thread**; the node is
spun by an executor in another thread; blocking helpers wait on `threading.Event`s set by
callbacks (`common/ros_utils.py`). `nav2_simple_commander.BasicNavigator` instead spins *itself*
inside its calls — `mission_runner` shows both styles side by side.

## Debugging toolkit

| Tool | Use |
|---|---|
| `ros2 node list`, `ros2 node info /amcl` | who is running, what they publish/subscribe |
| `ros2 topic list / hz / echo / info -v` | data flowing? rates? QoS mismatch? |
| `rqt_graph` | picture of nodes and topics |
| `ros2 run tf2_tools view_frames` | the TF tree as a PDF (missing link = broken chain) |
| `ros2 run tf2_ros tf2_echo A B`, `tf2_monitor` | one transform / who publishes it and how often |
| `ros2 param get /node name`, `ros2 param dump /node` | effective parameters (did my YAML load?) |
| `ros2 lifecycle get /node` | is a managed node active? |
| `ros2 action list -t`, `ros2 action send_goal ... --feedback` | talk to Nav2 directly |
| RViz | costmaps, plans, particles, frontiers, scenario obstacles, trails |
| `~/nav_robot_runs/<run>/*.log` | full output of each stage |

Classic traps: missing `use_sim_time` (timestamps from two different clocks), frame name
mismatch (`base_link` vs `base_footprint`), two publishers of one transform, wrong LiDAR
orientation in the URDF, wheel separation in the drive plugin ≠ URDF.
