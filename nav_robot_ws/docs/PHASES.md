# Build plan, phase by phase

Every phase has a **goal**, the **command(s)**, **what you should see**, a **verification
checklist** (things you can measure) and **experiments**. Don't move on until the checks pass —
later phases depend on earlier ones, and a bug found late is a bug that is hard to find.

Tip: keep a short log per phase ("what broke, how I fixed it"). It becomes your write-up.

Conventions used below
* T1, T2 … = separate terminals. Every terminal needs `source ~/nav_robot_ws/install/setup.bash`.
* Add `sim:=lite` to any phase launch to use the lightweight simulator instead of Gazebo.
* Results of every run go to `~/nav_robot_runs/<timestamp>/`.

---

## Phase 0 — Environment

**Goal:** ROS 2 Jazzy + Gazebo Harmonic + Nav2 + slam_toolbox installed; the workspace builds.

```bash
ros2 run demo_nodes_cpp talker          # T1
ros2 run demo_nodes_cpp listener        # T2 -> "I heard: Hello World"
gz sim shapes.sdf                       # a Gazebo window with shapes (skip if you use sim:=lite)
cd ~/nav_robot_ws && colcon build --symlink-install && source install/setup.bash
cd src/my_bot_autonomy && python3 -m pytest test -q
```

- [ ] talker/listener talk
- [ ] `gz sim shapes.sdf` opens (or you decided to use `sim:=lite`)
- [ ] `ros2 pkg list | grep -E "nav2_bt_navigator|slam_toolbox|ros_gz_bridge"` shows all three
- [ ] `colcon build` finishes with 0 errors; `pytest` shows 16 passed

---

## Phase 1 — Robot model (URDF/xacro) and TF tree

**Goal:** a correct digital body, every frame in the right place.

```bash
ros2 launch my_bot_description display.launch.py
ros2 run tf2_tools view_frames                         # T2: writes frames_<date>.pdf
ros2 run tf2_ros tf2_echo base_link lidar_link          # T2
xacro $(ros2 pkg prefix my_bot_description)/share/my_bot_description/urdf/my_bot.urdf.xacro > /tmp/my_bot.urdf
check_urdf /tmp/my_bot.urdf
```

Read: `my_bot_description/urdf/my_bot.urdf.xacro` (start here) → `inc/properties.xacro` → `inc/base.xacro` → `inc/sensors/*.xacro`.

What you should see: RViz shows the blue body, black wheels, red LiDAR, white camera; a slider
window moves the wheels.

- [ ] `check_urdf` prints "Successfully Parsed XML" and the tree `base_footprint → base_link → …`
- [ ] RViz shows no red errors (Fixed Frame `base_footprint`)
- [ ] Moving a wheel slider spins that wheel
- [ ] `tf2_echo base_link lidar_link` → Translation `[0.050, 0.000, 0.105]`
- [ ] The camera chain is `base_link → lidar_link → camera_link → camera_optical_link`, and the
      optical frame has z pointing forward (`tf2_echo base_link camera_optical_link`: RPY ≈ [-90°, 0, -90°])
- [ ] The LiDAR's red x-axis points forward, like `base_link`'s

Experiments
* Change `wheel_separation` in `properties.xacro` and look how both the URDF and (Phase 2) the
  drive plugin change — one source of truth.
* Try `xacro ... drive:=ros2_control` and `use_camera:=true` and diff the outputs.

---

## Phase 2 — Gazebo world, sensors, driving

**Goal:** the robot lives in the room, publishes realistic sensor data, and you can drive it.

```bash
ros2 launch my_bot_bringup phase2_sim.launch.py                 # T1 (Gazebo + RViz)
ros2 run teleop_twist_keyboard teleop_twist_keyboard            # T2 (keep this window focused)
ros2 topic hz /scan ; ros2 topic hz /odom ; ros2 topic echo /clock --once    # T3
```

Read: `my_bot_gazebo/launch/sim.launch.py`, `config/bridge_gz_plugin.yaml`, `worlds/nav_room.sdf`,
`my_bot_bringup/launch/robot.launch.py`, `my_bot_autonomy/robot/odometry_processor.py`.

What you should see: in RViz (Fixed Frame `odom`) red LiDAR points on the walls, a **green
trail** (ground truth) and a **yellow trail** (odometry). Drive around: the yellow trail slowly
separates from the green one — that is odometry drift (`odometry_profile:=realistic`).

- [ ] `/scan` ≈ 10 Hz, `/odom` ≈ 50 Hz, `/clock` ticking
- [ ] Forward key moves forward, left key turns left (not reversed!)
- [ ] Drive 2 m straight: `/odom` reports 2 m ± 5 cm (± ~3 cm extra with the realistic profile)
- [ ] Spin in place 360°: LiDAR points stay on the walls (Fixed Frame `odom`) — smearing = wrong TF or timestamps
- [ ] `view_frames` shows `odom → base_footprint → base_link → …`
- [ ] `ros2 run my_bot_autonomy sim_world spawn box1 1.0 0.0` makes a box appear in front of the robot

Experiments
* `odometry_profile:=perfect` vs `terrible`: compare the yellow trail after the same drive.
* `drive:=ros2_control`: then `ros2 control list_controllers` (two active controllers).
* `use_ekf:=true odometry_profile:=terrible`: the EKF fuses the gyro → much less heading drift.
* `sim:=lite`: same topics, no Gazebo. `ros2 topic list` is identical.

---

## Phase 3 — SLAM: build and save a map

**Goal:** a clean, accurate map, built while driving by keyboard, saved to disk.

```bash
ros2 launch my_bot_bringup phase3_slam.launch.py                # T1
ros2 run teleop_twist_keyboard teleop_twist_keyboard            # T2: drive slowly through BOTH zones, back to start
ros2 run my_bot_autonomy save_map --ros-args -p output:=$HOME/maps/my_room        # T3
ros2 run my_bot_autonomy compare_maps --map $HOME/maps/my_room.yaml --world nav_room.sdf
```

Read: `my_bot_navigation/launch/slam.launch.py`, `config/slam/slam_toolbox_async.yaml`,
`my_bot_autonomy/mapping/map_saving.py`, `compare_maps.py`.

- [ ] TF now has `map → odom → base_footprint`
- [ ] `my_room.pgm`, `my_room.yaml` (resolution 0.05) and `my_room.posegraph`/`.data` exist
- [ ] Walls are single thin lines (no doubled/ghost walls)
- [ ] RViz "Measure" tool: each outer wall ≈ 10 m ± 10 cm, corners ≈ 90°
- [ ] `compare_maps` → `wall_error_p95 ≤ 0.10`, `ghost_wall_ratio ≤ 0.02`, `coverage ≥ 0.90` → **PASS**

Experiments
* `odometry_profile:=terrible`: drive a loop and watch the map **snap straight** when you return
  to the start (loop closure). Same drive with `slam_method:=learning_mapper` (no correction) → smeared.
* `slam_method:=slam_toolbox_sync` vs `async`; `slam_method:=cartographer` (after installing it).
* Run `ros2 run my_bot_autonomy learning_mapper --ros-args -p pose_source:=odom -p use_sim_time:=true`
  alongside and show `/learning/map` in RViz: the map you get WITHOUT SLAM.

---

## Phase 4 — Localization on the saved map

**Goal:** with SLAM off, the robot finds itself on a known map from a rough initial guess.

```bash
ros2 launch my_bot_bringup phase4_localization.launch.py                              # reference map
ros2 launch my_bot_bringup phase4_localization.launch.py map:=$HOME/maps/my_room.yaml  # your map
ros2 run teleop_twist_keyboard teleop_twist_keyboard                                  # T2, drive ~2 m
ros2 topic echo /evaluation/localization_error                                        # T3
```

The robot spawns at (0, 0, 0) but AMCL is told (0.3, −0.2, 0.2) — deliberately wrong.

- [ ] The green particle cloud starts spread out
- [ ] After ≤ 2 m of driving it shrinks to a tight cluster on the robot
- [ ] Live LiDAR points sit on the map walls
- [ ] `/evaluation/localization_error` < 0.10 m (and `evaluation_summary.json` in the run folder)
- [ ] A worse guess (`init_x:=1.0 init_yaw:=0.5`) still converges
- [ ] Only ONE node publishes `map → odom` (`ros2 run tf2_ros tf2_monitor map odom`)

Experiments
* `amcl_profile:=beam_model`, `fast`, `many_particles` — convergence time and CPU (`htop`).
* `localization_method:=learning_pf` — our own 200-line particle filter (orange arrows).
* `localization_method:=slam_toolbox map:=<map saved by save_map>` — scan matching against the pose graph.
* Kidnap: `ros2 run my_bot_autonomy sim_world teleport my_bot -3 3 0` and watch AMCL get lost;
  then try again with `amcl_profile:=kidnap_recovery`.

---

## Phase 5 — Nav2: plan and drive to goals

**Goal:** click a goal, the robot gets there without touching anything — including obstacles
that are not in the map.

```bash
ros2 launch my_bot_bringup phase5_navigation.launch.py           # click "2D Goal Pose" in RViz
ros2 run my_bot_autonomy sim_world spawn box1 2.0 -1.0           # while it drives
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -4.2, y: -4.2}, orientation: {w: 1.0}}}}" --feedback
```

Read: `my_bot_navigation/launch/navigation.launch.py`, `config/nav2/nav2_base.yaml`,
`planners/*.yaml`, `controllers/*.yaml`, `behavior_trees/navigate_to_pose_default.xml`.

- [ ] Global costmap: walls with a coloured halo (inflation)
- [ ] Goals in all four corners and in the other zone (through a doorway) are reached within 25 cm / 14°
- [ ] Path keeps ≥ 15 cm from walls
- [ ] A box spawned on the path appears in the local costmap within 1 s; the robot goes around it
- [ ] After `sim_world remove box1` the box clears from the costmap
- [ ] A goal inside a wall is rejected cleanly (Nav2 reports failure, nothing crashes)

Experiments (one at a time, same goals, compare time and smoothness — see EXPERIMENTS.md)
* `planner:=navfn_astar | smac_2d | theta_star | smac_hybrid | smac_lattice`
* `controller:=dwb | mppi | graceful | rotation_shim`
* `use_collision_monitor:=false` and drive into a freshly spawned box
* Edit `inflation_radius` / `cost_scaling_factor` in `nav2_base.yaml` (no rebuild needed) and watch paths change.

---

## Phase 6 — Recovery from a hard block

**Goal:** when the path is completely blocked, the robot notices, tries recoveries, and
finally gives up gracefully (or succeeds if the block goes away).

```bash
ros2 launch my_bot_bringup phase6_recovery.launch.py                 # default tree
ros2 launch my_bot_bringup phase6_recovery.launch.py bt:=patient      # waits longer
ros2 launch my_bot_bringup phase6_recovery.launch.py bt:=no_recovery  # no recoveries at all
ros2 launch my_bot_bringup phase6_recovery.launch.py bt:=nav2_jazzy_default  # upstream tree with error-code gate
```

A cage of 4 walls closes around the robot 2 s after the goal starts and opens after 32 s
(`config/scenarios/hard_block_only.yaml`). Watch the terminal for `Running spin`, `Running wait`,
`Running backup`, `Aborting` and RViz for the robot turning in place.

- [ ] Planner reports no valid path; the behaviour tree clears the costmaps, spins, waits, backs up
- [ ] With `bt:=default`: the goal is ABORTED → the mission runner's retry succeeds after the cage opens
- [ ] With `bt:=patient`: often still trying when the cage opens → SUCCEEDED on the first attempt
- [ ] With `bt:=no_recovery`: fails immediately
- [ ] With `bt:=nav2_jazzy_default`: recoveries only run when the error code passes the gate
      (`WouldAPlannerRecoveryHelp`). In the cage the planner says "no valid path", which passes,
      so it behaves like `default`; with other errors (start/goal inside an obstacle) it aborts in
      about a second. Read the log for `error_code` to see which case you hit.
- [ ] `mission_results.csv` shows the result, duration and `recoveries` count for each attempt
- [ ] AMCL stays converged through spins and back-ups (`/evaluation/localization_error` < 0.1 m)

Also try the classic cases with `sim_world` while Phase 5 runs:
Case A (temporary block — remove the box after 15 s), Case B (block `door_east` with
`sim_world spawn plug 3.0 1.5 --size 1.4 0.4 0.8` while heading north-east → re-route through the west door).

---

## Phase 7 — Multi-goal mission

**Goal:** a queued list of goals runs end to end, handles failures, avoids a surprise
obstacle and logs results.

```bash
ros2 launch my_bot_bringup phase7_mission.launch.py
ros2 launch my_bot_bringup phase7_mission.launch.py mission_method:=follow_waypoints scenario_file:=none
ros2 launch my_bot_bringup phase7_mission.launch.py mission_file:=patrol_mission.yaml mission_method:=through_poses
ros2 run my_bot_autonomy run_report                                    # afterwards
```

Read: `my_bot_bringup/config/missions/demo_mission.yaml`, `scenarios/demo_scenario.yaml`,
`my_bot_autonomy/mission/mission_runner.py`, `scenario_director.py`.

- [ ] No RViz clicks needed at any point; the launch ends by itself
- [ ] AMCL converges from the rough guess during the initial spin
- [ ] The surprise box (goal 1) is avoided with zero near-collisions
- [ ] The east door gets blocked (goal 2) → re-route through the west door → success
- [ ] The cage (goal 3): first attempt FAILED, retry SUCCEEDED
- [ ] `mission_results.csv` has one row per attempt; `summary.md` lists the total mission time

---

## Phase 8 — Autonomous exploration

**Goal:** the robot maps the room by itself and saves the map.

```bash
ros2 launch my_bot_bringup phase8_exploration.launch.py
ros2 launch my_bot_bringup phase8_exploration.launch.py explore_strategy:=nearest
ros2 launch my_bot_bringup phase8_exploration.launch.py explorer:=reactive reactive_mode:=wall_follow
ros2 launch my_bot_bringup phase8_exploration.launch.py explorer:=explore_lite
```

Read: `my_bot_autonomy/exploration/frontiers.py` (the algorithm), `frontier_explorer.py` (the node).

Two lessons we learned the hard way while building this phase (both are fixed in the configs):
* slam_toolbox ignores pure rotation unless `check_min_dist_and_heading_precisely: true` —
  without it, the initial 360° spin adds nothing to the map. Try setting it back to `false`.
* Nav2 action servers exist before they are *active*; goals sent too early are rejected.
  The explorer waits for `/bt_navigator/get_state == active` (like `ros2 lifecycle get`).

- [ ] Starting from an empty map the robot covers both zones with no input
- [ ] Exploration ends by itself (no endless loop on tiny frontiers); robot returns to (0, 0)
- [ ] `explored_map.yaml/.pgm/.posegraph` in the run folder; `compare_maps` → PASS
- [ ] `exploration_result.json`: duration (target < 5 min in this room), goals reached/failed
- [ ] Zero collisions (`evaluation_summary.json` → `near_collisions`)

---

## Phase 9 — One command, unattended, timed

**Goal:** a single `ros2 launch` does everything; you get the wall-clock number.

```bash
ros2 launch my_bot_bringup full_mission.launch.py
ros2 launch my_bot_bringup full_mission_events.launch.py          # same pipeline, launch-events technique
```

- [ ] From a clean terminal, one command runs to completion with zero input
- [ ] 5 runs in a row succeed (fill the scorecard below from each `summary.md`)
- [ ] Each stage start/end is in the timeline of the report
- [ ] Nothing keeps running after the end (`ros2 node list` is empty)
- [ ] (portfolio) a screen recording of one run

| Run | Explore time (s) | Mission time (s) | Total (s) | Goals reached | Near-collisions |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |
| 4 | | | | | |
| 5 | | | | | |

---

## Bonus — the from-scratch stack

```bash
ros2 launch my_bot_bringup learning_stack.launch.py                  # our PF + A* + pure pursuit
ros2 launch my_bot_bringup learning_stack.launch.py algorithm:=dijkstra
```

Click "2D Goal Pose": the terminal prints a table comparing A*, Dijkstra and greedy search
(cells expanded, time, path length); RViz shows the orange path, the purple expanded cells,
the orange particle cloud of our particle filter, and `/learning/map` built from odometry only.
Read the `my_bot_autonomy/learning/*.py` files next to the Nav2 equivalents.
