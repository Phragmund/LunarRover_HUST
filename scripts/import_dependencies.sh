#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u

mkdir -p "${ROOT_DIR}/src/third_party" "${ROOT_DIR}/third_party" "${ROOT_DIR}/.deps"
vcs import "${ROOT_DIR}/src/third_party" < "${ROOT_DIR}/dependencies/core.repos"
vcs import "${ROOT_DIR}/src/third_party" < "${ROOT_DIR}/dependencies/fast_lio2.repos"
vcs import "${ROOT_DIR}/third_party" < "${ROOT_DIR}/dependencies/sdk.repos"

cmake -S "${ROOT_DIR}/third_party/Livox-SDK2" \
  -B "${ROOT_DIR}/.deps/livox-sdk2-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${ROOT_DIR}/.deps/livox-sdk2"
# Livox-SDK2's default "all" target builds sample tools that CI does not use.
# Build only the SDK libraries, then install the headers and libraries needed by ROS packages.
cmake --build "${ROOT_DIR}/.deps/livox-sdk2-build" \
  --target livox_lidar_sdk_static livox_lidar_sdk_shared \
  --parallel "${LIVOX_SDK2_BUILD_PARALLEL:-2}"
cmake --install "${ROOT_DIR}/.deps/livox-sdk2-build"

echo "Raw sensor dependencies are pinned and ready."
