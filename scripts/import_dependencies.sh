#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u

mkdir -p "${ROOT_DIR}/src/third_party" "${ROOT_DIR}/third_party" "${ROOT_DIR}/.deps"
vcs import "${ROOT_DIR}/src/third_party" < "${ROOT_DIR}/dependencies/core.repos"
vcs import "${ROOT_DIR}/third_party" < "${ROOT_DIR}/dependencies/sdk.repos"

IKD_TREE_DIR="${ROOT_DIR}/src/third_party/FAST_LIO/include/ikd-Tree"
if [[ ! -f "${IKD_TREE_DIR}/ikd_Tree.h" || ! -f "${IKD_TREE_DIR}/ikd_Tree.cpp" ]]; then
  git -C "${ROOT_DIR}/src/third_party/FAST_LIO" submodule update --init --recursive
fi

if ! git -C "${ROOT_DIR}/src/third_party/FAST_LIO" apply --check \
  "${ROOT_DIR}/patches/fast_lio_humble.patch" >/dev/null 2>&1; then
  if ! git -C "${ROOT_DIR}/src/third_party/FAST_LIO" apply --reverse --check \
    "${ROOT_DIR}/patches/fast_lio_humble.patch" >/dev/null 2>&1; then
    echo "FAST_LIO patch does not apply to the pinned source." >&2
    exit 1
  fi
else
  git -C "${ROOT_DIR}/src/third_party/FAST_LIO" apply \
    "${ROOT_DIR}/patches/fast_lio_humble.patch"
fi

cmake -S "${ROOT_DIR}/third_party/Livox-SDK2" \
  -B "${ROOT_DIR}/.deps/livox-sdk2-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${ROOT_DIR}/.deps/livox-sdk2"
cmake --build "${ROOT_DIR}/.deps/livox-sdk2-build" --parallel
cmake --install "${ROOT_DIR}/.deps/livox-sdk2-build"

echo "Core dependencies are pinned and ready."
