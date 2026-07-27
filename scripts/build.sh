#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u
export CMAKE_PREFIX_PATH="${ROOT_DIR}/.deps/livox-sdk2:${CMAKE_PREFIX_PATH:-}"
cd "${ROOT_DIR}"
rosdep install \
  --from-paths \
    src/lunar_rover_msgs \
    src/lunar_rover_description \
    src/lunar_rover_simulation \
    src/lunar_rover_control \
    src/lunar_rover_sensors \
    src/lunar_rover_bringup \
  --ignore-src --rosdistro humble -r -y
colcon build \
  --packages-select \
    livox_ros_driver2 \
    orbbec_camera_msgs \
    orbbec_description \
    orbbec_camera \
    lunar_rover_msgs \
    lunar_rover_description \
    lunar_rover_simulation \
    lunar_rover_control \
    lunar_rover_sensors \
    lunar_rover_bringup \
  --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo
