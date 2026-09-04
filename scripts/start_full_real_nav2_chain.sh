#!/usr/bin/env bash
# One-button full real-rover chain for the ROS unified web console.
set -euo pipefail

repo_root=/home/phragmund/LunarRover
start_mapping="${1:-false}"
cd "$repo_root"
exec >>/tmp/start_full_real_nav2_chain.log 2>&1

echo "=== Unified console requested complete real Nav2 chain ==="
bash "$repo_root/scripts/restart_real_nav2.sh"
if [[ "$start_mapping" != "true" ]]; then
  echo "=== Standby is ready; waiting for phone mapping start ==="
  exit 0
fi
echo "=== Standby is ready; now starting mapping/navigation ==="
exec bash "$repo_root/scripts/start_real_mapping_navigation.sh"
