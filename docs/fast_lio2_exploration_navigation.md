# FAST-LIO2 terrain exploration navigation

This stack is the 3D LiDAR-IMU route for autonomous mapping, exploration, and
navigation. It is independent of the older 2D `slam_toolbox` launch path.

## Dependencies

Import the FAST-LIO2 ROS 2 backend before launching the full stack:

```bash
vcs import src/third_party < dependencies/fast_lio2.repos
colcon build --symlink-install
source install/setup.bash
```

The wrapper defaults to a package named `fast_lio` and launch file
`mapping.launch.py`. Override those if the imported backend uses different
names:

```bash
ros2 launch lunar_rover_bringup fast_lio2_exploration_navigation.launch.py \
  fast_lio_package:=fast_lio \
  fast_lio_launch_file:=mapping.launch.py
```

## Full Simulation Stack

```bash
ros2 launch lunar_rover_bringup fast_lio2_exploration_navigation.launch.py \
  mode:=sim \
  world:=reachable_maze \
  gui:=true \
  start_exploration:=true
```

Main data flow:

```text
/livox/points + /livox/imu
  -> FAST-LIO2
  -> /localization/lio_odometry
  -> map_odom_tf_bridge + /localization/composite_odometry
  -> map -> odom -> base_link
  -> terrain_mapper -> /terrain/traversability
  -> Nav2 TerrainLayer
  -> frontier_explorer -> NavigateToPose
  -> twist_stamper -> /chassis/cmd_vel
```

For wrapper-only debugging when FAST-LIO2 is not imported yet:

```bash
ros2 launch lunar_rover_localization fast_lio2_localization.launch.py \
  run_fast_lio2:=false
```

## Key Topics

- `/localization/lio_odometry`: global FAST-LIO2 pose in `map`.
- `/localization/composite_odometry`: local wheel-distance + IMU-heading odom.
- `/terrain/traversability`: 2.5D terrain risk grid consumed by Nav2.
- `/terrain/diagnostics`: compact terrain mapper status.
- `/navigate_to_pose` and `/compute_path_to_pose`: Nav2 actions used by exploration.

Ground truth topics under `/simulation/ground_truth/*` are for evaluation only
and must not be consumed by localization, mapping, exploration, or navigation
nodes.
