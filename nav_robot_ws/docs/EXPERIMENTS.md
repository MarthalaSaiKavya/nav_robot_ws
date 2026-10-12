# Guided experiments

Each experiment changes **one** thing and compares numbers that the project measures for you
(`summary.md`, `mission_results.csv`, `evaluation_summary.json`, `map_quality.json`).
Use `gui:=false rviz:=false` for timing runs (rendering steals CPU), and run each configuration
2-3 times — simulations have noise too.

Shortcut used below: `FM = ros2 launch my_bot_bringup full_mission.launch.py gui:=false`
and `P7 = ros2 launch my_bot_bringup phase7_mission.launch.py gui:=false`.

---

### E1 — Why SLAM? (odometry vs mapping vs SLAM)
```bash
ros2 launch my_bot_bringup phase3_slam.launch.py odometry_profile:=terrible
ros2 run my_bot_autonomy learning_mapper --ros-args -p pose_source:=odom -p use_sim_time:=true
```
Drive one loop through both doorways. In RViz compare `/map` (slam_toolbox) and `/learning/map`
(same scans, odometry poses only). Save both and run `compare_maps` on each.
*Expect:* odometry-only map has bent/doubled walls (ghost ratio ≫ 2 %); SLAM map passes.

### E2 — SLAM implementations
`slam_method:=slam_toolbox_async | slam_toolbox_sync | cartographer` with the same keyboard route
(or with Phase 8 exploration). Record `map_quality.json` and CPU (`htop`).

### E3 — Exploration strategies
```bash
for s in nearest largest utility information_gain random; do
  ros2 launch my_bot_bringup phase8_exploration.launch.py gui:=false rviz:=false explore_strategy:=$s
done
```
Compare `exploration_result.json` (duration, goals sent, distance) and map coverage.
Then `explorer:=reactive reactive_mode:=wall_follow` and `random_walk`: notice what a reactive
explorer never sees (the free-standing obstacles in the middle).

### E4 — Localization from a rough guess
```bash
P7 initial_pose_mode:=topic          # rough guess (default)
P7 initial_pose_mode:=global         # no guess at all - may lock onto the wrong, symmetric spot!
P7 localization_method:=learning_pf  # our particle filter
P7 amcl_profile:=beam_model
```
Compare `localization_error_m` (mean / p95) in `evaluation_summary.json` and the per-goal
`localization_error_m` column.

### E5 — Planners
```bash
ros2 launch my_bot_bringup phase5_navigation.launch.py planner:=<name>
```
Send the same 3 goals (`ros2 action send_goal …`) for each planner. Watch the `/plan` shape:
NavFn (staircase-ish), Theta* (straight segments), Smac Hybrid (wide car-like turns),
Smac Lattice (smooth primitives). Also run `learning_stack.launch.py` to see expansions per algorithm.

### E6 — Controllers
```bash
P7 mission_file:=quick_mission.yaml scenario_file:=surprise_only.yaml controller:=<rpp|dwb|mppi|graceful|rotation_shim>
```
Compare mission time, `near_collisions`, and how each handles the surprise box
(RPP stops and waits for the planner; DWB/MPPI swerve locally).

### E7 — Behaviour trees and recovery
```bash
ros2 launch my_bot_bringup phase6_recovery.launch.py bt:=<default|nav2_jazzy_default|patient|no_recovery|replan_on_invalid>
```
Compare attempts, `recoveries` and total time in `mission_results.csv`.
Change the `open_cage` delay in `hard_block_only.yaml` to 15 s or 90 s and repeat.

### E8 — Sensor fusion
```bash
P7 odometry_profile:=terrible use_ekf:=false
P7 odometry_profile:=terrible use_ekf:=true
```
Compare `odometry_drift_m` and `localization_error_m`.

### E9 — Ways to send goals
```bash
P7 mission_file:=patrol_mission.yaml scenario_file:=none mission_method:=<navigate_to_pose|follow_waypoints|through_poses|raw_action>
```
Compare mission time (through_poses never stops) and robustness (through_poses fails as a whole).

### E10 — Tuning by hand (no rebuild needed with --symlink-install)
* `config/nav2/nav2_base.yaml`: `inflation_radius`, `cost_scaling_factor`, `robot_radius`
* `controllers/rpp.yaml`: `desired_linear_vel`, `lookahead_dist`
* `config/localization/amcl.yaml`: `alpha1..4`, `max_particles`, `laser_model_type`
* `config/slam/slam_toolbox_async.yaml`: `minimum_travel_distance`, `do_loop_closing`
Write down: change → effect. That table is the most valuable artefact of the project.

### E11 — Orchestration techniques
`full_mission.launch.py` vs `full_mission_events.launch.py`: same pipeline, different structure.
Read both; break one stage on purpose (e.g. `slam_method:=cartographer` without installing it)
and see which one tells you more clearly what went wrong.
