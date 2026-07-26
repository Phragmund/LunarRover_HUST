# Lunar rover simulation

This package supplies deterministic Gazebo Fortress environments and the
ROS-Gazebo bridge for the six-wheel rover.

```bash
ros2 launch lunar_rover_simulation simulation.launch.py world:=mixed
```

Available worlds are `flat`, `steps`, `rocks`, and `mixed`. All worlds use
standard Earth gravity (`9.80665 m/s^2`), local assets only, and deterministic
seed `360335` by default.

Headless execution is supported without an X server:

```bash
ros2 launch lunar_rover_simulation simulation.launch.py \
  world:=mixed gui:=false headless:=true rviz:=false
```

`simulation.launch.py` starts only the joint-state broadcaster. The integrated
bringup passes `lunar_rover_control/config/controllers.yaml` through the
`controller_config` argument and starts the four-steer/six-drive chassis and
rocker differential controllers from `lunar_rover_control`.
