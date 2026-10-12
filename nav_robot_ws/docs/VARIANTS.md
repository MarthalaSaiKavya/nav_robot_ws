# Techniques & variants — what each option does and when to use it

Almost every layer of a mobile-robot stack can be solved in several ways. This project ships
the common ones side by side, selected by **launch arguments**, so you can compare them on the
same robot, same room, same mission. This page explains each option.

Rule of thumb for experiments: **change one thing at a time**, run the same mission, compare the
numbers in `summary.md` (see [EXPERIMENTS.md](EXPERIMENTS.md)).

---

## 1. Simulator — `sim:=gazebo | lite`

| | Gazebo Sim (Harmonic) | lite_sim (this project) |
|---|---|---|
| What | Full 3D physics engine, rendered sensors | 100-line-per-feature Python loop: unicycle kinematics + ray-cast LiDAR |
| Pros | Realistic contacts, wheel slip, any sensor (camera, depth), the industry standard | Runs anywhere (no GPU), starts in 1 s, deterministic, readable code |
| Cons | Needs a GPU for the LiDAR, heavy | No dynamics, 2D only, no camera |
| Code | `my_bot_gazebo/worlds/*.sdf`, `config/bridge_*.yaml` | `my_bot_autonomy/sim/lite_sim.py` |

Both publish exactly the same topics (`/scan /imu /wheel/odom_raw /joint_states /clock
/ground_truth/odom`) and provide the same services (`/world/nav_room/create|remove|set_pose`),
so **nothing else changes**. That is the point: a well-designed robot stack doesn't know or care
what produces its sensor data. (Other simulators people use with Nav2: Gazebo Classic (EOL),
Webots, Isaac Sim, Stage/Flatland, and Nav2's own `nav2_loopback_sim`.)

## 2. Wheel drive — `drive:=gz_plugin | ros2_control`

* **`gz_plugin`** — Gazebo's `DiffDrive` system plugin reads `/cmd_vel` (through the bridge),
  turns the wheels and publishes odometry. Simplest.
* **`ros2_control`** — the professional architecture: a *hardware interface* (here
  `gz_ros2_control/GazeboSimSystem`; on a real robot your motor driver) + generic, well-tested
  *controllers* (`diff_drive_controller`, `joint_state_broadcaster`) managed by `controller_manager`.
  Switching from simulation to a real robot = changing one `<plugin>` line.
  Jazzy's `diff_drive_controller` only accepts `TwistStamped`, so a tiny `twist_stamper` node converts.
  Try `ros2 control list_controllers`, `ros2 control list_hardware_interfaces`.

## 3. Odometry errors — `odometry_profile:=realistic | perfect | terrible`

Simulated wheels are too good. `odometry_processor` re-integrates the wheel velocities with a
classic error model: **scale error** (wrong wheel radius), **rotation error** (wrong wheel
separation — the one that bends maps), **noise** and **bias**. Profiles live in
`my_bot_bringup/config/odometry/`. Make your own and pass its file name.

## 4. Sensor fusion — `use_ekf:=false | true`

`robot_localization`'s Extended Kalman Filter fuses wheel velocities (good at distance) with the
IMU gyro (good at rotation) into a better `odom → base_footprint`. Config:
`my_bot_navigation/config/odometry/ekf.yaml`. Strongest effect with `odometry_profile:=terrible`.
(Alternatives: UKF node in the same package; `fuse` (factor graph) for advanced users.)

---

## 5. SLAM — `slam_method:=...`

| Option | Technique | Notes |
|---|---|---|
| `slam_toolbox_async` (default) | Graph SLAM (Karto scan matcher + Ceres optimiser), drops scans when busy | Nav2's default SLAM. Keeps real time on any CPU. |
| `slam_toolbox_sync` | Same, processes every scan | More consistent, more CPU, can lag. |
| `cartographer` | Google Cartographer: local submaps + global pose graph, branch-and-bound loop closure | Excellent loop closure, heavy Lua configuration. Needs `ros-jazzy-cartographer-ros`. |
| `learning_mapper` | Our log-odds occupancy mapping with **known (ground truth) poses** | Not SLAM! Shows what mapping looks like when localization is solved. Compare with `pose_source:=odom` to see why SLAM exists. |

All four provide `/map` + `map → odom`. Others you will meet: GMapping (particle-filter SLAM,
ROS 1), Hector SLAM (scan matching without odometry), KISS-ICP / LIO-SAM / FAST-LIO (3D LiDAR).

**Map saving** has three implementations too (`my_bot_autonomy/mapping/map_saving.py`):
Nav2's `map_saver` service, slam_toolbox's `save_map` service, and a 30-line Python writer — plus
`serialize_map` which saves slam_toolbox's **pose graph** (needed for localization mode and to
continue mapping later).

## 6. Localization — `localization_method:=...`

| Option | Technique | Notes |
|---|---|---|
| `amcl` (default) | Adaptive Monte-Carlo Localization: particle filter + KLD-adaptive particle count | The standard. Tune with `amcl_profile:=`. |
| `slam_toolbox` | Scan matching against the saved pose graph | Robust to small map changes; needs `<map>.posegraph`. |
| `learning_pf` | Our particle filter (`learning/particle_filter.py`): likelihood field, odometry motion model, low-variance resampling, augmented-MCL recovery | ~200 lines, same idea as AMCL. |

AMCL profiles (`amcl_profile:=`): `default` (likelihood-field model), `beam_model` (ray-casting
sensor model), `many_particles`, `kidnap_recovery` (random particle injection), `fast`.

Initial pose (`initial_pose_mode:=`): `topic` (publish a rough guess on `/initialpose`, like RViz's
"2D Pose Estimate"), `params` (written into AMCL's parameters), `global` (particles everywhere +
spin — global localization, may fail in symmetric rooms!), `none`.

---

## 7. Global planner — `planner:=...`

All are Nav2 plugins (config in `my_bot_navigation/config/nav2/planners/`).

| Option | Search | Good for |
|---|---|---|
| `navfn_dijkstra` (default) | Dijkstra wavefront on the costmap | Robust classic, circular robots |
| `navfn_astar` | NavFn with A* heuristic | Same paths, fewer expansions |
| `smac_2d` | Modern cost-aware 2D A* + smoother | Recommended NavFn replacement |
| `theta_star` | Any-angle A* (line-of-sight shortcuts) | Straighter paths in open space |
| `smac_hybrid` | Hybrid-A* over (x, y, θ) with Dubins/Reeds-Shepp motions | Car-like robots; on diff-drive: wide turns (instructive) |
| `smac_lattice` | State lattice with precomputed diff-drive motion primitives | Non-circular robots in tight spaces |

See them think: `my_bot_autonomy/learning/grid_planners.py` implements Dijkstra/A*/greedy;
`learning_planner` prints how many cells each expands for your goal.

## 8. Controller (local planner) — `controller:=...`

Config in `config/nav2/controllers/`.

| Option | Technique | Character |
|---|---|---|
| `rpp` (default) | Regulated Pure Pursuit: geometric tracking of a look-ahead point, speed regulated by curvature, obstacles and goal distance | Smooth, predictable, follows the path exactly, stops for obstacles (planner re-routes) |
| `dwb` | Dynamic Window Approach: sample (v, ω), simulate, score with critics | Very tunable, deviates around obstacles locally |
| `mppi` | Model Predictive Path Integral: 2000 sampled control sequences per cycle, weighted by cost | Nav2 Jazzy's default; smoothest and best with dynamic obstacles; CPU-hungry |
| `graceful` | Graceful control law (Park & Kuipers) | Gentle human-like arcs, few parameters |
| `rotation_shim` | Rotate in place toward the path first, then hand over to RPP | Composing plugins; avoids S-curves at the start |

From scratch: `learning/pure_pursuit.py` (+ `learning_controller` node).

## 9. Behaviour tree — `bt:=...`

The "brain" that sequences planning, following and recovery (`my_bot_navigation/behavior_trees/`).

| Option | Behaviour |
|---|---|
| `default` | Replan at 1 Hz; on any failure run one recovery in turn — clear costmaps → spin → wait → back up — and retry, up to 6 times (the classic Nav2 structure) |
| `nav2_jazzy_default` | Nav2 Jazzy's own default: same, but recoveries only run when the error code says they could help (`WouldAPlannerRecoveryHelp` / `WouldAControllerRecoveryHelp`). "No valid path" passes the gate (so in the Phase 6 cage it behaves like `default`); errors like "start/goal occupied" do not, and then it aborts in about a second |
| `patient` | Waits first (10 s), larger back-up, full spin, 10 retries — good when blockers move away |
| `no_recovery` | Plan + follow only: fails at the first problem (an experiment to see what recoveries buy) |
| `replan_on_invalid` | Keep the current path while it stays valid; replan only when blocked or the goal changes |

Plus `use_collision_monitor:=true|false`: an independent safety layer between the controller and
the wheels that slows down / stops based on the raw scan.

---

## 10. Exploration — `explorer:=...`, `explore_strategy:=...`

| explorer | How it explores |
|---|---|
| `frontier` (default) | Our `frontier_explorer`: frontier detection → clustering → scoring → Nav2 goal; blacklists unreachable spots; returns home; saves the map |
| `explore_lite` | The community m-explore-ros2 node drives (build from source); our node watches and finishes the job. It starts after a `wait_for_nav2` gate (Nav2 active + one 360° spin) — explore_lite aims at frontier *centroids*, and on an empty map the only frontier is a ring whose centroid is the robot itself |
| `reactive` | No map, no planner: `wall_follow` (a FIND_WALL → FOLLOW state machine, right-hand rule) or `random_walk` (Roomba style). Publishes `/cmd_vel` itself — Nav2 is not even started |

Frontier strategies (`explore_strategy:=`): `nearest`, `largest`, `utility`
(explore_lite's `potential_scale·distance − gain_scale·size`), `information_gain` (unknown
area around the goal per metre), `random` (baseline). `use_path_distance:=true` ranks by
real planned path length instead of straight-line distance.
Frontier **segmentation**: frontiers longer than `max_frontier_length` (3 m, explorer
parameter; 0 = off) are cut into pieces with k-means. Without it, the first frontier is one
ring around the robot whose centroid is the robot itself — a useless goal.
Others in the literature: RRT-exploration, next-best-view with ray-traced information gain,
TSP-ordered frontiers (e.g. FAEL/FUEL), learned exploration policies.

## 11. Sending goals — `mission_method:=...`

| Option | API | Trade-off |
|---|---|---|
| `navigate_to_pose` (default) | `BasicNavigator.goToPose()` per goal | Full control: per-goal timeouts, retries, events |
| `follow_waypoints` | `BasicNavigator.followWaypoints()` → Nav2 `waypoint_follower` | Least code; stops at each waypoint; skips failures |
| `through_poses` | `BasicNavigator.goThroughPoses()` | One smooth path through all poses; one bad pose fails all |
| `raw_action` | plain `rclpy` ActionClient (`common/nav2_actions.py`) | Shows what the helper hides: goal / feedback / result / cancel |

## 12. Orchestration — who runs the stages?

* `full_mission.launch.py` — a **supervisor node** runs each stage as its own `ros2 launch`
  process, waits for it, teleports the robot, collects results. Easy to read and debug.
* `full_mission_events.launch.py` — **pure launch events**: `OnProcessExit` of the explorer →
  `ShutdownProcess` for every stage-1 process → teleport → `TimerAction` → include stage 2;
  `OnShutdown` writes the report. No extra node, but the logic is spread over callbacks.
* (Not included, also common: a **behaviour tree / state machine** for the mission itself —
  e.g. a custom Nav2 BT, SMACH/FlexBE-style state machines, or `py_trees`.)

## 13. Lifecycle management — two techniques in the code

* Nav2 servers, AMCL, map_server, map_saver → `nav2_lifecycle_manager` (with bond heartbeats)
* slam_toolbox → **launch events** (`LifecycleNode` + `ChangeState` events), as upstream does
  (see `slam_toolbox_lifecycle_node()` in `my_bot_autonomy/launch_utils.py`).

## 14. Changing the world — two backends

`SimWorldClient` (`common/sim_services.py`): `backend='ros'` uses ROS services
(`ros_gz_interfaces`, provided by the bridge or lite_sim); `backend='gz_cli'` shells out to
`gz service`. Try `ros2 run my_bot_autonomy sim_world --backend gz_cli spawn b 1 1`.

## 15. From-scratch learning implementations

| Module | Algorithm | Real-world counterpart |
|---|---|---|
| `learning/occupancy_mapping.py` | Log-odds occupancy grid, Bresenham / vectorised ray tracing | mapping part of slam_toolbox / gmapping |
| `learning/particle_filter.py` | MCL: odometry motion model, likelihood field, systematic resampling, augmented MCL | nav2_amcl |
| `learning/distance_transform.py` | Exact EDT (Felzenszwalb–Huttenlocher) and brushfire | likelihood field, costmap inflation |
| `learning/grid_planners.py` | Dijkstra, A*, greedy best-first, inflation cost, line-of-sight smoothing | NavFn, Smac 2D, Theta* |
| `learning/pure_pursuit.py` | (Regulated) pure pursuit | Nav2 RPP |
| `exploration/frontiers.py` | Frontier detection, clustering, 5 scoring strategies | explore_lite |
