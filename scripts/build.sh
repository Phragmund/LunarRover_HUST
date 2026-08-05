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
    src \
  --ignore-src --rosdistro humble -r -y
colcon build \
  --packages-select \
    livox_ros_driver2 \
    fast_lio \
    orbbec_camera_msgs \
    orbbec_description \
    orbbec_camera \
    lunar_rover_msgs \
    lunar_rover_description \
    lunar_rover_simulation \
    lunar_rover_control \
    lunar_rover_sensors \
    lunar_rover_localization \
    lunar_rover_mapping \
    lunar_rover_navigation \
    lunar_rover_exploration \
    lunar_rover_bringup \
  --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo
