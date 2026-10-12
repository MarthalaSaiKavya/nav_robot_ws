"""
my_bot_autonomy - every Python node of the project, organised by topic:

    common/       shared helpers (geometry, world parsing, map files, simulator services)
    sim/          the lightweight 2D simulator (alternative to Gazebo)
    robot/        "driver-level" nodes: odometry processing, Twist -> TwistStamped
    mapping/      map saving, reference map generation, map comparison
    exploration/  autonomous exploration (frontier-based + reactive variants)
    mission/      multi-goal mission runner, scenario (obstacle) director, supervisor
    evaluation/   ground-truth based metrics and run reports
    learning/     from-scratch implementations of mapping, particle filter, A*, pure pursuit

Pure-Python algorithm modules (no ROS imports) live next to the nodes that use them,
so you can unit-test them and read them without the ROS boilerplate.
"""
