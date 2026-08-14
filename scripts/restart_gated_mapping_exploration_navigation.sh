#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p log/manual_start
echo "$$" > log/manual_start/latest_gated.pid
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

wait_for_topic_once() {
  local topic="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))
  local remaining probe_timeout topic_type
  echo "[$(date +%T)] waiting for $topic, timeout ${timeout_sec}s"
  while (( SECONDS < deadline )); do
    topic_type="$(ros2 topic type "$topic" 2>/dev/null || true)"
    if [[ -n "$topic_type" ]]; then
      remaining=$((deadline - SECONDS))
      probe_timeout=3
      if (( remaining < probe_timeout )); then
        probe_timeout="$remaining"
      fi
      if timeout "$probe_timeout" ros2 topic echo --once "$topic" >/dev/null 2>/dev/null; then
        echo "[$(date +%T)] received first $topic message"
        return 0
      fi
    fi
    sleep 1
  done
  echo "Timed out waiting for topic message: $topic" >&2
  return 1
}

wait_for_controllers_active() {
  local timeout_sec="$1"
  local deadline=$((SECONDS + timeout_sec))
  local output
  echo "[$(date +%T)] waiting for rover controllers, timeout ${timeout_sec}s"
  while (( SECONDS < deadline )); do
    output="$(timeout 5 ros2 control list_controllers 2>/dev/null || true)"
    if grep -q "joint_state_broadcaster.*active" <<< "$output" &&
      grep -q "rocker_differential_controller.*active" <<< "$output" &&
      grep -q "rover_controller.*active" <<< "$output"
    then
      echo "[$(date +%T)] rover controllers are active"
      return 0
    fi
    sleep 2
  done
  echo "Timed out waiting for rover controllers to become active" >&2
  echo "$output" >&2
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
cloud_fusion_enabled="${ENABLE_CLOUD_FUSION:-false}"
case "${cloud_fusion_enabled,,}" in
  1|true|yes|on)
    cloud_fusion_enabled=true
    ;;
  *)
    cloud_fusion_enabled=false
    ;;
esac

world="${WORLD:-reachable_maze}"
if [[ -n "${COLLISION_MARKER_Z_OFFSET:-}" ]]; then
  collision_marker_z_offset="$COLLISION_MARKER_Z_OFFSET"
else
  collision_marker_z_offset="0.20"
fi
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
if [[ -n "${MAP_ORIGIN_Z:-}" ]]; then
  map_origin_z="$MAP_ORIGIN_Z"
  map_origin_z_source="manual"
else
  map_origin_z="$spawn_z"
  map_origin_z_source="spawn_z"
fi
sim_settle_sec="${SIM_SETTLE_SEC:-12}"
if [[ "$world" == "image_cave" ]]; then
  clock_timeout_sec="${CLOCK_TIMEOUT_SEC:-180}"
  ground_truth_timeout_sec="${GROUND_TRUTH_TIMEOUT_SEC:-180}"
  controller_timeout_sec="${CONTROLLER_TIMEOUT_SEC:-180}"
else
  clock_timeout_sec="${CLOCK_TIMEOUT_SEC:-90}"
  ground_truth_timeout_sec="${GROUND_TRUTH_TIMEOUT_SEC:-90}"
  controller_timeout_sec="${CONTROLLER_TIMEOUT_SEC:-90}"
fi
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

echo "[$(date +%T)] starting sim: world=$world spawn_z=$spawn_z map_origin_z=$map_origin_z source=$map_origin_z_source delayed_livox_drivers=$delay_livox_drivers cloud_fusion=$cloud_fusion_enabled"
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

wait_for_topic_once /clock "$clock_timeout_sec"
wait_for_topic_once /simulation/ground_truth/odometry "$ground_truth_timeout_sec"
wait_for_controllers_active "$controller_timeout_sec"
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
  -r __node:=world_model_marker \
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

echo "[$(date +%T)] starting world collision marker z_offset=$collision_marker_z_offset"
ros2 run lunar_rover_simulation world_model_marker_node.py \
  --ros-args \
  -r __node:=world_collision_marker \
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
  -p marker_z_offset:="$collision_marker_z_offset" \
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
  params_file:=install/lunar_rover_mapping/share/lunar_rover_mapping/config/terrain_mapper.yaml \
  enable_cloud_fusion:="$cloud_fusion_enabled" &

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
