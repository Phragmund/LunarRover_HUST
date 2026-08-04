# Lunar rover simulation

This package supplies deterministic Gazebo Fortress environments and the
ROS-Gazebo bridge for the six-wheel rover.

```bash
ros2 launch lunar_rover_simulation simulation.launch.py
```

The default test world is `image_cave`. Available worlds include `flat`,
`steps`, `rocks`, `mixed`, `stairs_maze`, `reachable_maze`, `lava_tube`,
`image_cave`, and `terrain_preview`. All worlds use
standard Earth gravity (`9.80665 m/s^2`), local assets only, and deterministic
seed `360335` by default.

`image_cave` is the main full-system test map. It combines the generated cave
wall graph with the current triangular pseudo-surface terrain. See
`../../docs/simulation_worlds.md` for the complete world inventory and map
contents.

Headless execution is supported without an X server:

```bash
ros2 launch lunar_rover_simulation simulation.launch.py \
  gui:=false headless:=true rviz:=false
```

`simulation.launch.py` starts only the joint-state broadcaster. The integrated
bringup passes `lunar_rover_control/config/controllers.yaml` through the
`controller_config` argument and starts the four-steer/six-drive chassis and
rocker differential controllers from `lunar_rover_control`.
