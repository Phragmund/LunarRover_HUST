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

## Bridge and Sensor Gates

The simulation launch separates core simulator state from high-rate sensor
topics:

| File | Purpose |
| --- | --- |
| `config/bridge_base.yaml` | Always-on core bridges such as `/simulation/ground_truth/odometry` and collision state. |
| `config/bridge_sensors.yaml` | Delayed sensor bridges such as `/mid360/raw_points`, `/mid360/raw_imu`, and camera topics. |
| `config/bridge.yaml` | Legacy all-in-one bridge list kept for reference and ad hoc debugging. |

`simulation.launch.py` exposes two gates:

| Argument | Default | Effect |
| --- | --- | --- |
| `sensor_bridges` | `true` | Starts the Gazebo-to-ROS sensor bridge for Mid-360 and camera topics. |
| `livox_drivers` | `true` | Starts the Mid-360 ROS adapter nodes that publish `/livox/lidar`, `/livox/points`, and `/livox/imu`. |

The validated full-system startup path uses these gates to start Gazebo and
ground-truth odometry first, wait for the world to settle, then start the
sensor bridge and Livox drivers before FAST-LIO2. Use the root script
`scripts/restart_gated_mapping_exploration_navigation.sh` for that sequence.
