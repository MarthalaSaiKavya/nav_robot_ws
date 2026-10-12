"""
From-scratch implementations of the core algorithms behind SLAM/AMCL/Nav2.

They are deliberately simple, heavily commented and pure numpy, so you can read
them next to the "real" ROS packages and compare:

  occupancy_mapping.py  log-odds occupancy grid mapping + Bresenham ray tracing
  particle_filter.py    Monte-Carlo localization (what AMCL does)
  distance_transform.py exact Euclidean distance transform (likelihood field, costmap inflation)
  grid_planners.py      Dijkstra / A* / greedy best-first (what NavFn / Smac2D do)
  pure_pursuit.py       pure-pursuit path follower (the idea behind Regulated Pure Pursuit)

ROS wrappers that let you run them on the robot: learning_*_node.py
"""
