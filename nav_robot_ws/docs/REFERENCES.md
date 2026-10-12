# References

The configuration values, launch structure and plugin usage in this project were taken from (and
checked against the source code of) these projects and documents. Versions: ROS 2 Jazzy,
Nav2 1.3.x, slam_toolbox 2.8.x, ros_gz 1.0.x, Gazebo Harmonic.

## Official documentation
* Nav2 — [Setting Up Navigation Plugins (choosing planners/controllers)](https://docs.nav2.org/setup_guides/algorithm/select_algorithm.html)
* Nav2 — [Tuning Guide](https://docs.nav2.org/tuning/index.html)
* Nav2 — [Configuration Guide (Jazzy)](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/)
* Nav2 — [First-Time Robot Setup Guide, Gazebo (Jazzy)](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/gazebo/)
* Nav2 — [Setting Up Odometry – Gazebo](https://docs.nav2.org/setup_guides/odom/setup_odom_gz.html)
* Nav2 — [Smoothing Odometry using Robot Localization](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html)
* Nav2 — [Navigation plugins list](https://docs.nav2.org/plugins/index.html)
* Gazebo — [ROS 2 Gazebo vendor packages (Jazzy ↔ Harmonic)](https://gazebosim.org/docs/all/ros2_gz_vendor_pkgs/)
* ROS 2 — [Jazzy installation (Ubuntu)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
* REP-103 (units, conventions) and REP-105 (coordinate frames for mobile platforms) — https://www.ros.org/reps/

## Source repositories studied
* [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) (branch `jazzy`): `nav2_bringup` params, launch files, behaviour trees, plugin parameter declarations, Simple Commander
* [ros-navigation/nav2_minimal_turtlebot_simulation](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation) (branch `jazzy`): Gazebo Harmonic robot SDF, bridge YAML, spawn launch
* [SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) (branch `jazzy`): parameters, lifecycle launch, services
* [gazebosim/ros_gz](https://github.com/gazebosim/ros_gz) (1.0.24): bridge YAML format (topics + services), `create` tool
* [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim) (`gz-sim8`): DiffDrive, OdometryPublisher, JointStatePublisher, UserCommands plugin parameters
* [ros-controls/gz_ros2_control](https://github.com/ros-controls/gz_ros2_control) and [ros2_controllers](https://github.com/ros-controls/ros2_controllers) (branch `jazzy`): diff drive demo, `diff_drive_controller` parameters (TwistStamped-only in Jazzy)
* [cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization) (`jazzy-devel`)
* [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2): explore_lite frontier cost
* [ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3): `turtlebot3_cartographer` Lua configuration
* [joshnewans/articubot_one](https://github.com/joshnewans/articubot_one) (Articulated Robotics): xacro structure and inertial macros style

## Papers and books (the algorithms)
* B. Yamauchi, *A Frontier-Based Approach for Autonomous Exploration*, CIRA 1997 — frontier exploration
* [Frontier Based Exploration for Autonomous Robot (arXiv:1806.03581)](https://arxiv.org/pdf/1806.03581)
* S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press 2005 — log-odds mapping (ch. 9), MCL / AMCL, motion models (ch. 5), likelihood field (ch. 6)
* P. Felzenszwalb, D. Huttenlocher, *Distance Transforms of Sampled Functions*, Theory of Computing 2012
* S. Macenski, I. Jambrecic, *SLAM Toolbox: SLAM for the dynamic world*, JOSS 2021 — [paper](https://joss.theoj.org/papers/10.21105/joss.02783.pdf)
* W. Hess et al., *Real-Time Loop Closure in 2D LIDAR SLAM*, ICRA 2016 — Cartographer
* S. Macenski et al., *Regulated Pure Pursuit for Robot Path Tracking*, Autonomous Robots 2023
* G. Williams et al., *Information-Theoretic MPC / MPPI*, ICRA 2017 — MPPI
* J. J. Park, B. Kuipers, *A smooth control law for graceful motion of differential wheeled mobile robots in 2D environments*, ICRA 2011
* D. Fox, W. Burgard, S. Thrun, *The Dynamic Window Approach to Collision Avoidance*, IEEE RAM 1997 — DWB
* K. Daniel et al., *Theta\*: Any-Angle Path Planning on Grids*, JAIR 2010

## Comparisons and community write-ups
* [From Simulation to Reality: SLAM Toolbox vs Cartographer in ROS 2](https://avesis.iuc.edu.tr/yayin/d3614d16-298a-40f4-b89f-b1adfb243c06/from-simulation-to-reality-comparative-performance-analysis-of-slam-toolbox-and-cartographer-in-ros-2)
* [Comparative benchmarking of 2D LiDAR SLAM algorithms with ROS 2 on Raspberry Pi 5](https://openaccess.istun.edu.tr/entities/publication/d1239799-7ccc-42f9-aab3-c351b7861bd9)
* [rbot: an open-source AMR simulation stack for ROS 2 Jazzy and Gazebo Harmonic](https://discourse.openrobotics.org/t/rbot-an-open-source-amr-simulation-stack-for-ros-2-jazzy-and-gazebo-harmonic/54694)
* [frontier_exploration_ros2 announcement](https://discourse.openrobotics.org/t/frontier-exploration-ros2-modern-autonomous-exploration-system-for-ros-2/57222)
