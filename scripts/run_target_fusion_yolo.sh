#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -x "$repo_root/.deps/target_perception_venv/bin/python" ]]; then
  py="$repo_root/.deps/target_perception_venv/bin/python"
elif [[ -x "$repo_root/.venv_yolo/bin/python" ]]; then
  py="$repo_root/.venv_yolo/bin/python"
else
  echo "No YOLO perception venv found." >&2
  echo "Run scripts/setup_target_perception.sh first, or restore .venv_yolo." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$repo_root/install/setup.bash"
set -u

cd "$repo_root"
exec "$py" "$repo_root/src/lunar_rover_manipulation/scripts/target_fusion_detector_node.py" \
  --ros-args \
  --params-file "$repo_root/src/lunar_rover_manipulation/config/grasping.yaml" \
  -p open_vocab_enabled:=true \
  -p sam_enabled:="${SAM_ENABLED:-true}" \
  "$@"
