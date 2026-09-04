#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

h7_port="${REAL_CHASSIS_H7_PORT:-${REAL_CHASSIS_STEERING_PORT:-/dev/ttyACM0}}"
web_port="${WEB_JOYSTICK_PORT:-8000}"
web_token="${WEB_JOYSTICK_TOKEN:-}"
if [[ -z "$web_token" ]]; then
  web_token="$(python3 -c 'import secrets; print(secrets.token_hex(12))')"
fi

run_chassis="${RUN_REAL_CHASSIS:-true}"
run_rviz="${RUN_RVIZ:-true}"
run_grasping="${RUN_GRASPING:-true}"
detector_backend="${TARGET_DETECTOR_BACKEND:-auto}"
detector_model="${TARGET_DETECTOR_MODEL:-${TARGET_SEGMENTATION_MODEL:-}}"
active_search="${ACTIVE_TARGET_SEARCH:-true}"
auto_trigger_grasp="${AUTO_TRIGGER_GRASP:-false}"
periodic_sweep="${PERIODIC_ARM_SWEEP:-false}"
run_foundationpose_bridge="${RUN_FOUNDATIONPOSE_BRIDGE:-false}"
run_keyboard="${RUN_KEYBOARD:-true}"
lidar_frame_id="${REAL_LIDAR_FRAME_ID:-base_link}"
lidar_output_topic="${REAL_LIDAR_OUTPUT_TOPIC:-/livox/lidar}"
lio_body_frame="${REAL_LIO_BODY_FRAME:-base_link}"
arm_camera_xyz="${ARM_CAMERA_XYZ:-0.04 0 0.065}"
arm_camera_rpy="${ARM_CAMERA_RPY:-0 -1.57079632679 0}"

if [[ "$run_chassis" == "true" ]]; then
  REAL_CHASSIS_H7_PORT="$h7_port" scripts/check_real_chassis.sh
fi

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export LUNAR_ROVER_ROOT="$repo_root"

ip="$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -Ev '^(192\.168\.2\.50|127\.)' | head -n1)"
if [[ -z "$ip" ]]; then
  ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
if [[ -n "$ip" ]]; then
  echo "Phone controller: http://$ip:$web_port/?token=$web_token"
fi
echo "Real Livox mapping uses lidar_frame_id=$lidar_frame_id, lio_body_frame=$lio_body_frame"
echo "Real Livox point cloud topic: $lidar_output_topic"
echo "Keep the rover still until FAST-LIO level correction reports calibrated."
if [[ "$run_grasping" == "true" ]]; then
  echo "Autonomous grasping enabled: press C once after calibration and target setup."
fi
if [[ "$run_chassis" == "true" ]]; then
  echo "Using H7 port: $h7_port"
  echo "WARNING: motion_solver enables the 3508 drive on startup. Do not touch the phone joystick until the area is clear."
fi

exec ros2 launch "$repo_root/src/lunar_rover_bringup/launch/real_lidar_mapping.launch.py" \
  run_chassis:="$run_chassis" \
  run_rviz:="$run_rviz" \
  run_grasping:="$run_grasping" \
  detector_backend:="$detector_backend" \
  segmentation_model:="$detector_model" \
  active_search:="$active_search" \
  auto_trigger_grasp:="$auto_trigger_grasp" \
  periodic_sweep:="$periodic_sweep" \
  run_foundationpose_bridge:="$run_foundationpose_bridge" \
  run_keyboard:="$run_keyboard" \
  lidar_frame_id:="$lidar_frame_id" \
  lidar_output_topic:="$lidar_output_topic" \
  lio_body_frame:="$lio_body_frame" \
  h7_port:="$h7_port" \
  web_port:="$web_port" \
  web_access_token:="$web_token" \
  arm_camera_xyz:="$arm_camera_xyz" \
  arm_camera_rpy:="$arm_camera_rpy" \
  "$@"
