# Autonomous Navigation Robot — ROS 2 Jazzy · Gazebo Harmonic · SLAM · Nav2

A complete, beginner-friendly, **end-to-end** autonomous mobile robot project. A small
differential-drive robot with a 360° LiDAR is dropped into a 10 × 10 m room it has never
seen. With **one command** it:

1. **explores and maps** the room by itself (SLAM + frontier exploration),
2. **saves** the map,
3. is "kidnapped" to another spot and **re-localizes** from a rough guess (AMCL),
4. runs a **5-goal mission** while the world fights back:
   a surprise box appears on its path, a doorway gets blocked, and finally a cage closes
   around it (path fully occluded) so it must **detect the block, run recoveries and retry**,
5. prints a **report** with the total wall-clock time, map quality and localization error.

Every stage can be swapped for an **alternative technique** with a single launch argument
(6 planners, 5 controllers, 5 behaviour trees, 4 SLAM options, 3 localizers, 5 exploration
strategies + 2 other explorers, 4 ways to send goals, 2 simulators, 2 drive systems, EKF on/off …),
and the core algorithms are also **re-implemented from scratch in plain Python** so you can
see what's inside the black boxes.

```
ros2 launch my_bot_bringup full_mission.launch.py
```

---

## Contents

| Doc | What's inside |
|---|---|
| **README.md** (this file) | Install, build, run, the package map, the variant cheat-sheet, troubleshooting |
| [docs/PHASES.md](docs/PHASES.md) | The build plan, phase by phase (0 → 9): goal, commands, what you should see, **verification checklist**, experiments |
| [docs/VARIANTS.md](docs/VARIANTS.md) | Every alternative technique explained: how it works, when to use it, how to switch |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Nodes, topics, TF tree, the velocity-command chain, lifecycle, launch-file structure |
| [docs/EXPERIMENTS.md](docs/EXPERIMENTS.md) | Guided experiments that compare techniques with real numbers |
| [docs/REFERENCES.md](docs/REFERENCES.md) | The docs, repos, papers and articles this project is based on |

---

## 1. Requirements

| | |
|---|---|
| OS | **Ubuntu 24.04** (Noble) — native, dual-boot or a VM with 3D acceleration |
| ROS | **ROS 2 Jazzy Jalisco** (`ros-jazzy-desktop`) |
| Simulator | **Gazebo Harmonic** (installed automatically with `ros-jazzy-ros-gz`) — *or* the built-in lightweight simulator (`sim:=lite`, no GPU needed) |
| Hardware | 4+ CPU cores, 8 GB RAM recommended. A GPU helps Gazebo (the LiDAR is rendered). |

No Ubuntu machine? Every phase also runs with `sim:=lite` (pure Python 2D simulator) which
needs no GPU, so a modest VM is enough.

## 2. Install (once)

```bash
# --- ROS 2 Jazzy itself: follow https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
#     (sudo apt install ros-jazzy-desktop) then:

sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-tf2-tools \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-yaml python3-matplotlib python3-pytest

# Optional extras (only for the variants that use them)
sudo apt install -y ros-jazzy-cartographer-ros        # slam_method:=cartographer
```

## 3. Build

```bash
cd ~ && unzip ~/Downloads/nav_robot_ws.zip      # -> ~/nav_robot_ws/{README.md, docs/, src/my_bot_*}
cd ~/nav_robot_ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y       # installs anything still missing
colcon build --symlink-install
source install/setup.bash
```

Add the two `source` lines to your `~/.bashrc` so every new terminal is ready:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/nav_robot_ws/install/setup.bash" >> ~/.bashrc
```

`--symlink-install` means you can edit any YAML/launch/Python file in `src/` and simply re-run
— no rebuild needed (except after adding new files or entry points).

**Optional — explore_lite** (the community frontier explorer, `explorer:=explore_lite`) has no
Jazzy binary, build it from source once:

```bash
cd ~/nav_robot_ws/src && git clone https://github.com/robo-friends/m-explore-ros2.git
cd ~/nav_robot_ws && colcon build --symlink-install --packages-select explore_lite_msgs explore_lite
```

## 4. Run

### The single command (Phase 9 — the deliverable)

```bash
ros2 launch my_bot_bringup full_mission.launch.py                       # Gazebo window + RViz
ros2 launch my_bot_bringup full_mission.launch.py gui:=false rviz:=false # fastest, headless
ros2 launch my_bot_bringup full_mission.launch.py sim:=lite             # no Gazebo / no GPU
```

What you will see (RViz): the map growing as the robot explores (blue dots = frontiers,
green = chosen frontier), the robot returning home, then teleporting, the green AMCL
particle cloud collapsing onto it, and the 5-goal mission with orange obstacles appearing.
When it finishes the whole launch exits and prints a report like:

```
# Run report - 20261012_103000          (a real run, sim:=lite on a slow 2-core VM)
**Result: SUCCESS - all goals complete**
**Total wall-clock time (launch -> all goals complete): 558.3 s**
## Stage 1 - exploration + mapping
- strategy: `utility`, duration 223.8 s, distance driven: 31.3 m, returned home: True
## Map quality vs. the true world
| wall error mean | 0.024 m | coverage | 97.5 % | pass | True |
## Stage 2 - localization + mission
- method `navigate_to_pose`: 5/5 goals, mission time 177.2 s
| # | goal             | attempt | result    | time [s] | recoveries |
| 0 | zone_a_southwest | 1       | SUCCEEDED | 20.08    | 0  |
| 1 | zone_a_east      | 1       | SUCCEEDED | 33.82    | 0  |  <- surprise box on the way
| 2 | zone_b_northeast | 1       | SUCCEEDED | 39.84    | 2  |  <- east door blocked: re-route
| 3 | zone_b_west      | 1       | FAILED    | 20.41    | 15 |  <- the cage: Nav2 gave up as designed
| 3 | zone_b_west      | 2       | SUCCEEDED | 44.75    | 8  |  <- cage opened, retry succeeded
| 4 | home             | 1       | SUCCEEDED | 17.32    | 8  |
- localization error at the goals 0.05-0.11 m; odometry-only drift up to 2.5 m; near-collisions 0
```

Everything is saved in `~/nav_robot_runs/<timestamp>/`:

| File | Content |
|---|---|
| `summary.md`, `trajectory.png`, `errors.png` | the report and plots |
| `explored_map.yaml/.pgm` | the map built by SLAM (+ `.posegraph/.data` for slam_toolbox) |
| `exploration_result.json`, `map_quality.json` | stage 1 numbers |
| `mission_results.csv/.json` | per-goal result, time, recoveries, distance, final error |
| `pose_log.csv`, `evaluation_summary.json` | ground truth vs. estimate vs. odometry, every 0.2 s |
| `run_summary.json`, `stage1_explore.log`, `stage2_mission.log` | timeline and full logs |

Re-print a report any time: `ros2 run my_bot_autonomy run_report` (newest run) or
`ros2 run my_bot_autonomy run_report ~/nav_robot_runs/<timestamp>`.

### Phase by phase (learn it the way it is built)

| Phase | Command | You learn |
|---|---|---|
| 1 Robot model | `ros2 launch my_bot_description display.launch.py` | URDF, links/joints, TF tree |
| 2 Simulation | `ros2 launch my_bot_bringup phase2_sim.launch.py` + `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | Gazebo, bridge, sensors, odometry drift |
| 3 SLAM | `ros2 launch my_bot_bringup phase3_slam.launch.py` (+ teleop, then `ros2 run my_bot_autonomy save_map ...`) | mapping, loop closure, saving maps |
| 4 Localization | `ros2 launch my_bot_bringup phase4_localization.launch.py` (+ teleop) | AMCL, particle filters |
| 5 Navigation | `ros2 launch my_bot_bringup phase5_navigation.launch.py` (click "2D Goal Pose") | costmaps, planners, controllers |
| 6 Recovery | `ros2 launch my_bot_bringup phase6_recovery.launch.py` | behaviour trees, recoveries |
| 7 Mission | `ros2 launch my_bot_bringup phase7_mission.launch.py` | multi-goal missions, dynamic obstacles |
| 8 Exploration | `ros2 launch my_bot_bringup phase8_exploration.launch.py` | frontier exploration |
| 9 Everything | `ros2 launch my_bot_bringup full_mission.launch.py` | system integration |
| Bonus | `ros2 launch my_bot_bringup learning_stack.launch.py` | your own mapper / particle filter / A* / pure pursuit |

Each phase, its verification checklist and experiments are in **[docs/PHASES.md](docs/PHASES.md)**.

---

## 5. What's in the box

```
nav_robot_ws/src/
├── my_bot_description/     PHASE 1  robot model (xacro): body, wheels, LiDAR, IMU, camera,
│   ├── urdf/my_bot.urdf.xacro        drive variants (Gazebo DiffDrive | ros2_control), ground truth
│   ├── urdf/inc/*.xacro               one small documented file per part
│   ├── config/ros2_controllers.yaml   diff_drive_controller settings (drive:=ros2_control)
│   └── launch/display.launch.py       RViz + joint sliders
├── my_bot_gazebo/          PHASE 2  the world + simulator launch
│   ├── worlds/nav_room.sdf            10x10 m room, 2 zones joined by 2 doorways, obstacles
│   ├── worlds/nav_room_simple.sdf     same room, no partition (easier)
│   ├── config/bridge_*.yaml           ROS <-> Gazebo bridge (topics + world services)
│   └── launch/sim.launch.py           Gazebo or lite sim, spawn, bridge, controllers
├── my_bot_navigation/      PHASES 3-6 SLAM / localization / Nav2 configuration
│   ├── config/slam/                   slam_toolbox async/sync/localization, cartographer.lua
│   ├── config/localization/           amcl.yaml + amcl_profiles/
│   ├── config/nav2/                   nav2_base.yaml + planners/*.yaml + controllers/*.yaml
│   ├── config/odometry/ekf.yaml       robot_localization EKF
│   ├── behavior_trees/*.xml           5 behaviour trees
│   ├── maps/                          perfect reference maps of both worlds
│   └── launch/{slam,localization,navigation}.launch.py
├── my_bot_bringup/         PHASES 2-9 top-level launch files + missions + RViz layouts
│   ├── launch/robot.launch.py         simulator + odometry pipeline (+EKF) + evaluator
│   ├── launch/phaseN_*.launch.py      one per phase
│   ├── launch/stage_explore|stage_mission.launch.py   the two halves of the full run
│   ├── launch/full_mission.launch.py  THE single command (supervisor node)
│   ├── launch/full_mission_events.launch.py   same, built from launch events only
│   ├── launch/learning_stack.launch.py        the from-scratch stack
│   ├── config/missions/*.yaml         demo / quick / patrol / recovery-test missions
│   ├── config/scenarios/*.yaml        surprise obstacles, blocked doors, cage
│   └── config/odometry/*.yaml         odometry error profiles: perfect / realistic / terrible
└── my_bot_autonomy/        all Python nodes (ros2 run my_bot_autonomy <name>)
    ├── sim/lite_sim.py                lightweight 2D simulator (drop-in for Gazebo)
    ├── robot/                         odometry_processor (drift model), twist_stamper
    ├── mapping/                       save_map, make_reference_map, compare_maps
    ├── exploration/                   frontiers.py (algorithms), frontier_explorer, reactive_explorer
    ├── mission/                       mission_runner, scenario_director, supervisor
    ├── evaluation/                    evaluator (ground truth), run_report
    ├── learning/                      log-odds mapping, particle filter, A*/Dijkstra, pure pursuit (+ nodes)
    ├── common/                        geometry, world model, map files, Nav2 action helpers, sim services
    └── test/test_algorithms.py        18 unit tests (no ROS needed)
```

### Python commands (`ros2 run my_bot_autonomy <command>`)

| Command | Purpose |
|---|---|
| `save_map --ros-args -p output:=$HOME/maps/room` | Save the current SLAM map (+ pose graph) |
| `compare_maps --map room.yaml --world nav_room.sdf` | Score a map against the true world |
| `make_reference_map --world nav_room.sdf --out ~/maps/ref` | Perfect map from the SDF |
| `sim_world spawn box1 1.5 -1.0` / `remove box1` / `teleport my_bot 1 -4 3.14` | Change the world (Gazebo or lite) |
| `run_report [run_dir]` | Rebuild the report of a run |
| `wait_for_nav2` | Exits when Nav2 is active (a gate for launch files; optional `initial_spin`) |
| `frontier_explorer`, `reactive_explorer`, `mission_runner`, `scenario_director`, `supervisor`, `evaluator`, `lite_sim`, `odometry_processor`, `twist_stamper` | Nodes (normally started by launch files) |
| `learning_mapper`, `learning_localizer`, `learning_planner`, `learning_controller` | From-scratch algorithm nodes |

---

## 6. Variant cheat-sheet (all are launch arguments)

Pass any of these to the phase launch files or to `full_mission.launch.py`.
Full explanations: **[docs/VARIANTS.md](docs/VARIANTS.md)**.

| Layer | Argument | Options (default first) |
|---|---|---|
| Simulator | `sim` | `gazebo` · `lite` |
| Gazebo window | `gui` | `true` · `false` |
| World | `world` | `nav_room.sdf` · `nav_room_simple.sdf` · any path |
| Wheel drive | `drive` | `gz_plugin` · `ros2_control` |
| Odometry errors | `odometry_profile` | `realistic` · `perfect` · `terrible` |
| Sensor fusion | `use_ekf` | `false` · `true` (robot_localization EKF: wheels + IMU) |
| Camera | `use_camera` | `false` · `true` |
| SLAM | `slam_method` | `slam_toolbox_async` · `slam_toolbox_sync` · `cartographer` · `learning_mapper` |
| Localization | `localization_method` | `amcl` · `slam_toolbox` · `learning_pf` |
| AMCL tuning | `amcl_profile` | `default` · `beam_model` · `many_particles` · `kidnap_recovery` · `fast` |
| Global planner | `planner` | `navfn_dijkstra` · `navfn_astar` · `smac_2d` · `theta_star` · `smac_hybrid` · `smac_lattice` |
| Controller | `controller` | `rpp` · `dwb` · `mppi` · `graceful` · `rotation_shim` |
| Behaviour tree | `bt` | `default` · `nav2_jazzy_default` · `patient` · `no_recovery` · `replan_on_invalid` |
| Safety layer | `use_collision_monitor` | `true` · `false` |
| Explorer | `explorer` | `frontier` · `explore_lite` · `reactive` |
| Frontier strategy | `explore_strategy` | `utility` · `nearest` · `largest` · `information_gain` · `random` |
| Reactive mode | `reactive_mode` | `wall_follow` · `random_walk` |
| Goal sending | `mission_method` | `navigate_to_pose` · `follow_waypoints` · `through_poses` · `raw_action` |
| Initial pose | `initial_pose_mode` | `topic` · `params` · `global` · `none` |
| Mission | `mission_file` | `demo_mission.yaml` · `quick_mission.yaml` · `patrol_mission.yaml` · `recovery_test_mission.yaml` |
| Scenario | `scenario_file` | `demo_scenario.yaml` · `surprise_only.yaml` · `hard_block_only.yaml` · `none` |
| Orchestration | launch file | `full_mission.launch.py` (supervisor) · `full_mission_events.launch.py` (launch events) |

Example combinations:

```bash
ros2 launch my_bot_bringup full_mission.launch.py explore_strategy:=nearest planner:=smac_2d controller:=mppi
ros2 launch my_bot_bringup full_mission.launch.py odometry_profile:=terrible use_ekf:=true amcl_profile:=many_particles
ros2 launch my_bot_bringup full_mission.launch.py localization_method:=slam_toolbox mission_method:=follow_waypoints
ros2 launch my_bot_bringup phase5_navigation.launch.py drive:=ros2_control controller:=dwb bt:=replan_on_invalid
```

---

## 7. Tests

```bash
cd ~/nav_robot_ws/src/my_bot_autonomy && python3 -m pytest test -q       # 18 tests, ~5 s, no ROS needed
# (if pytest complains about a "launch_testing" plugin: add  -p no:launch_testing)
```

They check the world parser and ray casting, map file round-trips, the distance transform,
frontier detection, log-odds mapping quality, particle-filter convergence, A* vs Dijkstra,
pure pursuit, and that every mission goal is reachable.

## 8. Troubleshooting

| Symptom | Cause → fix |
|---|---|
| `Package 'my_bot_bringup' not found` | You did not `source ~/nav_robot_ws/install/setup.bash` in this terminal. |
| Gazebo opens but the robot never appears / `/scan` is silent | GPU/driver problem with the rendered LiDAR. Try `export LIBGL_ALWAYS_SOFTWARE=1` before launching, or use `sim:=lite`. In a VM enable 3D acceleration. |
| Gazebo is very slow (real-time factor ≪ 1) | Use `gui:=false` (server only), close RViz (`rviz:=false`), or `sim:=lite`. |
| `Timed out waiting for transform ... map` / Nav2 says "waiting for transform" | Localization not running or no initial pose: give one in RViz ("2D Pose Estimate") or use `set_initial_pose:=true`. |
| `Message Filter dropping message: frame 'lidar_link' ... earlier than all the data in the transform cache` | A node is not using sim time. All our nodes set `use_sim_time:=true`; for your own nodes add `--ros-args -p use_sim_time:=true`. |
| Robot jumps around in RViz | Two nodes publish `map → odom` (e.g. SLAM and AMCL together) or `odom → base_footprint`. Check with `ros2 run tf2_tools view_frames`. |
| Goals fail with "Timed out while waiting for action server to acknowledge goal request" | Very slow machine. Increase `default_server_timeout` in `config/nav2/nav2_base.yaml` (we already use 500 ms instead of 20). |
| MPPI is slow / "Control loop missed its desired rate" | MPPI needs CPU. Lower `batch_size` / `time_steps` in `controllers/mppi.yaml` or use `controller:=rpp`. |
| Leftover nodes after Ctrl+C (strange behaviour in the next run) | `pkill -f ros2; pkill -f gz` then start again. `ros2 node list` should be empty. |
| `explore_lite` not found | Build it from source (section 3) or use `explorer:=frontier`. |
| `cartographer_ros` not found | `sudo apt install ros-jazzy-cartographer-ros`. |
| `slam_toolbox` localization: ".posegraph not found" | Use a map saved by the explorer or `save_map` (both also write the pose graph), not a hand-made PGM. |
| `Lookup would require extrapolation into the future` / `Transform data too old` from the controller, `Message Filter dropping message ... queue is full` from slam_toolbox | The PC cannot keep up and SLAM's `map → odom` lags behind the clock. Close RViz / use `gui:=false`, prefer `localization_method:=amcl`, or raise `transform_timeout` in the slam_toolbox YAML. (We developed on a 2-core VM where this happened; with 4+ cores it should not.) |
| Exploration ends after a few seconds with a tiny map | Nav2 was not active yet or SLAM ignored the initial spin — both fixed here (explorer waits for `/bt_navigator` = active; `check_min_dist_and_heading_precisely: true`). If you change those, expect this symptom. |

More debugging habits (and the tools to use) are in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md#debugging-toolkit).

## 9. How this was verified

Everything below was **run on ROS 2 Jazzy** (Nav2 1.3, slam_toolbox 2.8.5, explore_lite from
source) with `sim:=lite gui:=false rviz:=false` on a deliberately weak **2-core VM**. Every
launch file that is meant to finish did finish by itself with exit code 0.

| What | Result |
|---|---|
| Unit tests (`pytest`) | 18 / 18 pass |
| URDF, every xacro variant | `check_urdf` OK; TF: lidar at (0.05, 0, 0.105), camera optical frame rotated correctly |
| Phase 3 SLAM (scripted drive) | map PASS: wall error mean 0.026 m, p95 0.05 m, 0 % ghost walls |
| Phase 4 AMCL from a 0.36 m-off guess | converged to 0.005–0.07 m after a spin and a short drive |
| Phase 5, 2 goals each | SUCCEEDED with all 6 planners, all 5 controllers, BTs `patient` / `no_recovery` / `replan_on_invalid`, `use_ekf:=true`, no collision monitor, own particle filter |
| Phase 6 cage (hard block) | `default` / `nav2_jazzy_default`: ABORT then retry SUCCEEDED; `patient`: SUCCEEDED first try; `no_recovery`: FAILED (as intended) |
| Phase 7 quick mission (3 goals) | 3/3 with `follow_waypoints`, `through_poses`, `raw_action` (+ surprise box), `initial_pose_mode:=global`, `params`, `odometry_profile:=terrible use_ekf:=true`, `amcl_profile:=beam_model`, `localization_method:=slam_toolbox`* |
| Phase 8 exploration (all maps PASS vs. the true world) | utility 147 s · nearest 160 s · information_gain + `slam_toolbox_sync` 182 s · random + `use_path_distance` 193 s · largest + `learning_mapper` 213 s · explore_lite 333 s · reactive wall_follow / random_walk 240 s (coverage 94 % / 97 %) |
| Phase 9 `full_mission.launch.py` | **SUCCESS in both runs: 558 s and 594 s total**, 5/5 goals (explore 224 / 263 s, map PASS, mission 177 / 164 s, cage → retry) |
| Phase 9 `full_mission_events.launch.py` | **SUCCESS, 5/5 goals, 458 s total** |
| Learning stack | own particle filter + A* (and Dijkstra) + pure pursuit: (0, 0) → through a doorway → (-3.5, 3.0) in 32 s, localization error 1–2 cm |

\* `slam_toolbox` localization reached all goals, but on the overloaded 2-core VM it once
jumped 1.3 m along a featureless wall — a real weakness worth seeing (compare with AMCL).

**Not executed here** (no GPU / not installable in the build environment): the Gazebo path
(`sim:=gazebo`: world SDF, sensor + plugin tags, bridge YAML, spawning), `drive:=ros2_control`,
`use_camera:=true`, `slam_method:=cartographer` and the RViz/GUI windows. Those files follow
the official Nav2 Jazzy + Gazebo Harmonic reference simulation (`nav2_minimal_turtlebot_simulation`),
the ros_gz / gz_ros2_control sources and cartographer_ros' own examples. If something
Gazebo-specific misbehaves on your machine, `sim:=lite` runs the identical pipeline, and
section 8 lists the usual fixes.

**Bugs these tests caught** (each one is a good lesson — the fixes are commented in the code):
* Nav2 action servers exist before they are *active* → early goals are rejected
  (`common/nav2_actions.py: wait_until_active`).
* slam_toolbox ignores pure rotation by default → the explorer's initial spin added nothing
  (`check_min_dist_and_heading_precisely: true`).
* A frontier that rings the robot has its centroid *at* the robot → split long frontiers
  (`frontiers.py: split_cluster`); explore_lite gets a seeding spin instead (`wait_for_nav2`).
* Pure pursuit on a 3-corner path chased the start point behind it → resample the path
  (`pure_pursuit.py: densify`).
* `BasicNavigator` keeps the previous goal's feedback → recoveries were counted twice.
* explore_lite stops by cancelling *all* goals → it cancelled our "return home" goal.
* A wall follower with no wall in sight drove in circles forever → FIND_WALL / FOLLOW states.
* A map saved as PGM marks unknown cells 205; read with the wrong thresholds they become free.

License: Apache-2.0.
