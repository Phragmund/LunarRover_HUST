#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u

mkdir -p "${ROOT_DIR}/src/third_party" "${ROOT_DIR}/third_party" "${ROOT_DIR}/.deps"
vcs import "${ROOT_DIR}/src/third_party" < "${ROOT_DIR}/dependencies/core.repos"
vcs import "${ROOT_DIR}/third_party" < "${ROOT_DIR}/dependencies/sdk.repos"

cmake -S "${ROOT_DIR}/third_party/Livox-SDK2" \
  -B "${ROOT_DIR}/.deps/livox-sdk2-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${ROOT_DIR}/.deps/livox-sdk2"
cmake --build "${ROOT_DIR}/.deps/livox-sdk2-build" --parallel
cmake --install "${ROOT_DIR}/.deps/livox-sdk2-build"

echo "Raw sensor dependencies are pinned and ready."
