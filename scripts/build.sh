#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u
export CMAKE_PREFIX_PATH="${ROOT_DIR}/.deps/livox-sdk2:${CMAKE_PREFIX_PATH:-}"

cd "${ROOT_DIR}"
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build \
  --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS="${ROS_DISTRO:-humble}"
