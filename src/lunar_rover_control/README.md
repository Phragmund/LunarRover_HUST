# lunar_rover_control

This package contains only the low-level simulation-facing vehicle interfaces:

- `/chassis/cmd_vel` input as `geometry_msgs/msg/TwistStamped`;
- four steering position commands;
- six wheel velocity commands;
- passive rocker differential control;
- simulated `/chassis/state` feedback and command watchdog.

It intentionally contains no wheel odometry, pose estimation, filtering,
mapping, planning, obstacle avoidance, or autonomous mission logic.
