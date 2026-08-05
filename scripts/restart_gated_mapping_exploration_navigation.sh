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

echo "[$(date +%T)] starting sim/sensors only"
ros2 launch lunar_rover_bringup bringup.launch.py \
  mode:=sim \
  world:=reachable_maze \
  gui:=true \
  headless:=false \
  rviz:=false \
  sensor_noise:=false \
  spawn_x:=-3.92 \
  spawn_y:=0.0 \
  spawn_z:=0.20 \
  spawn_yaw:=0.0 &

echo "[$(date +%T)] starting composite odometry"
ros2 run lunar_rover_mapping composite_wheel_imu_odometry_node.py \
  --ros-args \
  -r __node:=composite_wheel_imu_odometry \
  --params-file install/lunar_rover_mapping/share/lunar_rover_mapping/config/composite_odometry.yaml &

echo "[$(date +%T)] starting world model marker and RViz"
ros2 run lunar_rover_simulation world_model_marker_node.py \
  --ros-args \
  -p use_sim_time:=true \
  -p world_name:=reachable_maze \
  -p frame_id:=map \
  -p marker_topic:=/simulation/world_model_markers \
  -p alpha:=0.30 \
  -p outline_alpha:=0.30 \
  -p outline_width:=0.025 \
  -p map_origin_world_x:=-3.92 \
  -p map_origin_world_y:=0.0 \
  -p map_origin_world_z:=0.20 \
  -p map_origin_world_yaw:=0.0 &

rviz2 -d install/lunar_rover_mapping/share/lunar_rover_mapping/rviz/mapping.rviz \
  --ros-args \
  -p use_sim_time:=true &

sleep 6

echo "[$(date +%T)] starting FAST-LIO localization; keep rover still until level correction completes"
ros2 launch lunar_rover_localization fast_lio2_localization.launch.py \
  run_fast_lio2:=true \
  direct_fast_lio_node:=true \
  run_tf_bridge:=true \
  run_map_frame_node:=true \
  run_input_normalizer:=false \
  run_sim_imu_adapter:=false \
  imu_mount_pitch_source:=startup_imu_pitch \
  imu_mount_pitch_samples:=80 \
  imu_mount_pitch_max_gyro_radps:=0.02 \
  level_correction_source:=startup_lio_pose \
  level_correction_samples:=40 \
  level_correction_start_delay_sec:=8.0 \
  level_correction_preserve_yaw:=true \
  level_correction_anchor_position:=true &

wait_for_topic_once /localization/lio_odometry 45
echo "[$(date +%T)] FAST-LIO corrected odometry is live; starting terrain mapping"

ros2 launch lunar_rover_mapping terrain_mapping.launch.py \
  params_file:=install/lunar_rover_mapping/share/lunar_rover_mapping/config/terrain_mapper.yaml &

sleep 2

echo "[$(date +%T)] starting Nav2 and command mux after level correction"
ros2 launch lunar_rover_navigation navigation.launch.py \
  params_file:=install/lunar_rover_navigation/share/lunar_rover_navigation/config/nav2_fast_lio2_params.yaml &

wait_for_lifecycle_active /bt_navigator 60

echo "[$(date +%T)] starting direct keyboard teleop"
scripts/run_direct_keyboard_teleop.sh \
  --no-acl \
  --device /dev/input/event3,/dev/input/event6,/dev/input/event9,/dev/input/event13 &

echo "[$(date +%T)] starting frontier exploration"
ros2 launch lunar_rover_exploration frontier_exploration.launch.py \
  params_file:=install/lunar_rover_exploration/share/lunar_rover_exploration/config/frontier_explorer.yaml &

wait
