#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u
mkdir -p "${ROOT_DIR}/src/third_party"
vcs import "${ROOT_DIR}/src/third_party" < "${ROOT_DIR}/dependencies/hardware.repos"
echo "Orbbec hardware sources imported. Install the vendor udev rules before use."
