#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash

wait_for_topic_once() {
  local topic="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))
  local remaining
  echo "[$(date +%T)] waiting for $topic, timeout ${timeout_sec}s"
  while (( SECONDS < deadline )); do
    if [[ -n "$(ros2 topic type "$topic" 2>/dev/null || true)" ]]; then
      remaining=$((deadline - SECONDS))
      timeout "${remaining}" ros2 topic echo --once "$topic" >/dev/null
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for topic type: $topic" >&2
  return 1
}

wait_for_lifecycle_active() {
  local node="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))
  echo "[$(date +%T)] waiting for lifecycle node $node to become active"
  while (( SECONDS < deadline )); do
    if ros2 lifecycle get "$node" 2>/dev/null | grep -q "active"; then
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for $node to become active" >&2
  return 1
}

nav2_enabled="${ENABLE_NAV2:-false}"
case "${nav2_enabled,,}" in
  1|true|yes|on)
    nav2_enabled=true
    ;;
  *)
    nav2_enabled=false
    ;;
esac

world="${WORLD:-reachable_maze}"
spawn_x="${SPAWN_X:--3.92}"
spawn_y="${SPAWN_Y:-0.0}"
spawn_yaw="${SPAWN_YAW:-0.0}"
if [[ -n "${SPAWN_Z:-}" ]]; then
  spawn_z="$SPAWN_Z"
elif [[ "$world" == "image_cave" ]]; then
  spawn_z="1.45"
else
  spawn_z="0.45"
fi
map_origin_z="${MAP_ORIGIN_Z:-$spawn_z}"
sim_settle_sec="${SIM_SETTLE_SEC:-12}"
delay_livox_drivers="${DELAY_LIVOX_DRIVERS:-true}"
case "${delay_livox_drivers,,}" in
  1|true|yes|on)
    delay_livox_drivers=true
    livox_drivers_at_bringup=false
    sensor_bridges_at_bringup=false
    ;;
  *)
    delay_livox_drivers=false
    livox_drivers_at_bringup=true
    sensor_bridges_at_bringup=true
    ;;
esac

echo "[$(date +%T)] starting sim: world=$world delayed_livox_drivers=$delay_livox_drivers"
ros2 launch lunar_rover_bringup bringup.launch.py \
  mode:=sim \
  world:="$world" \
  gui:=true \
  headless:=false \
  rviz:=false \
  sensor_noise:=false \
  livox_drivers:="$livox_drivers_at_bringup" \
  sensor_bridges:="$sensor_bridges_at_bringup" \
  spawn_x:="$spawn_x" \
  spawn_y:="$spawn_y" \
  spawn_z:="$spawn_z" \
  spawn_yaw:="$spawn_yaw" &

wait_for_topic_once /clock 60
wait_for_topic_once /simulation/ground_truth/odometry 60
echo "[$(date +%T)] Gazebo ground truth is live; waiting ${sim_settle_sec}s before Livox ROS drivers"
sleep "$sim_settle_sec"

if [[ "$delay_livox_drivers" == "true" ]]; then
  echo "[$(date +%T)] starting delayed Gazebo sensor bridge"
  ros2 run ros_gz_bridge parameter_bridge \
    --ros-args \
    -r __node:=simulation_sensor_bridge \
    -p use_sim_time:=true \
    -p config_file:=install/lunar_rover_simulation/share/lunar_rover_simulation/config/bridge_sensors.yaml &

  echo "[$(date +%T)] starting delayed Mid-360 ROS point cloud driver"
  ros2 run lunar_rover_simulation mid360_livox_sim_driver.py \
    --ros-args \
    -r __node:=mid360_livox_sim_driver \
    -p use_sim_time:=true \
    -p raw_points_topic:=/mid360/raw_points \
    -p raw_imu_topic:=/mid360/raw_imu \
    -p truth_odom_topic:=/simulation/ground_truth/odometry \
    -p imu_source:=ground_truth_kinematics \
    -p publish_imu:=false \
    -p custom_topic:=/livox/lidar \
    -p points_topic:=/livox/points \
    -p imu_topic:=/livox/imu_disabled \
    -p base_frame:=base_link \
    -p lidar_frame:=livox_frame \
    -p imu_frame:=livox_imu_frame \
    -p scan_rate:=10.0 \
    -p scan_lines:=11 &

  echo "[$(date +%T)] starting delayed Mid-360 IMU driver"
  ros2 run lunar_rover_simulation mid360_livox_imu_driver.py \
    --ros-args \
    -r __node:=mid360_livox_imu_driver \
    -p use_sim_time:=true \
    -p raw_imu_topic:=/mid360/raw_imu \
    -p truth_odom_topic:=/simulation/ground_truth/odometry \
    -p imu_topic:=/livox/imu \
    -p base_frame:=base_link \
    -p imu_frame:=livox_imu_frame &

  wait_for_topic_once /livox/lidar 45
  wait_for_topic_once /livox/imu 45
fi

echo "[$(date +%T)] starting composite odometry"
ros2 run lunar_rover_mapping composite_wheel_imu_odometry_node.py \
  --ros-args \
  -r __node:=composite_wheel_imu_odometry \
  --params-file install/lunar_rover_mapping/share/lunar_rover_mapping/config/composite_odometry.yaml &

echo "[$(date +%T)] starting world model marker and RViz"
ros2 run lunar_rover_simulation world_model_marker_node.py \
  --ros-args \
  -p use_sim_time:=true \
  -p world_name:="$world" \
  -p frame_id:=map \
  -p marker_topic:=/simulation/world_model_markers \
  -p alpha:=0.30 \
  -p outline_alpha:=0.30 \
  -p outline_width:=0.025 \
  -p map_origin_world_x:="$spawn_x" \
  -p map_origin_world_y:="$spawn_y" \
  -p map_origin_world_z:="$map_origin_z" \
  -p map_origin_world_yaw:="$spawn_yaw" &

echo "[$(date +%T)] starting world collision marker"
ros2 run lunar_rover_simulation world_model_marker_node.py \
  --ros-args \
  -p use_sim_time:=true \
  -p world_name:="$world" \
  -p frame_id:=map \
  -p marker_topic:=/simulation/world_collision_markers \
  -p include_visual:=false \
  -p include_collision:=true \
  -p marker_namespace:=world_collision \
  -p outline_namespace:=world_collision_outline \
  -p alpha:=0.16 \
  -p outline_alpha:=0.85 \
  -p outline_width:=0.018 \
  -p color_r:=0.05 \
  -p color_g:=0.85 \
  -p color_b:=1.0 \
  -p outline_color_r:=0.05 \
  -p outline_color_g:=0.95 \
  -p outline_color_b:=1.0 \
  -p map_origin_world_x:="$spawn_x" \
  -p map_origin_world_y:="$spawn_y" \
  -p map_origin_world_z:="$map_origin_z" \
  -p map_origin_world_yaw:="$spawn_yaw" &

rviz2 -d install/lunar_rover_mapping/share/lunar_rover_mapping/rviz/mapping.rviz \
  --ros-args \
  -p use_sim_time:=true &

sleep 6

echo "[$(date +%T)] starting FAST-LIO localization with fixed map leveling"
ros2 launch lunar_rover_localization fast_lio2_localization.launch.py \
  run_fast_lio2:=true \
  direct_fast_lio_node:=true \
  run_tf_bridge:=true \
  run_map_frame_node:=true \
  run_input_normalizer:=false \
  run_sim_imu_adapter:=false \
  imu_mount_pitch_source:=fixed \
  imu_mount_pitch_deg:=35.273 \
  level_correction_source:=fixed \
  level_correction_roll_deg:=-0.58 \
  level_correction_pitch_deg:=-0.67 \
  level_correction_yaw_deg:=0.007 &

wait_for_topic_once /localization/lio_odometry 45
echo "[$(date +%T)] FAST-LIO corrected odometry is live; starting terrain mapping"

ros2 launch lunar_rover_mapping terrain_mapping.launch.py \
  params_file:=install/lunar_rover_mapping/share/lunar_rover_mapping/config/terrain_mapper.yaml &

sleep 2

if [[ "$nav2_enabled" == "true" ]]; then
  echo "[$(date +%T)] starting Nav2 and command mux after level correction"
  ros2 launch lunar_rover_navigation navigation.launch.py \
    params_file:=install/lunar_rover_navigation/share/lunar_rover_navigation/config/nav2_fast_lio2_params.yaml &

  wait_for_lifecycle_active /bt_navigator 60

  echo "[$(date +%T)] starting direct keyboard teleop through command mux"
  scripts/run_direct_keyboard_teleop.sh \
    --no-acl &

  echo "[$(date +%T)] starting frontier exploration"
  ros2 launch lunar_rover_exploration frontier_exploration.launch.py \
    params_file:=install/lunar_rover_exploration/share/lunar_rover_exploration/config/frontier_explorer.yaml &
else
  echo "[$(date +%T)] skipping Nav2/frontier exploration; starting direct keyboard teleop on /chassis/cmd_vel"
  scripts/run_direct_keyboard_teleop.sh \
    --no-acl \
    --topic /chassis/cmd_vel &
fi

wait
