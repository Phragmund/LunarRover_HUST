#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
destination="$repo_root/.deps/FoundationPose"

if [[ -d "$destination/.git" ]]; then
  git -C "$destination" fetch --depth 1 origin main
  git -C "$destination" checkout --detach FETCH_HEAD
else
  git clone --depth 1 https://github.com/NVlabs/FoundationPose.git "$destination"
fi

cat <<EOF
FoundationPose source is available at:
  $destination

This host still needs a supported CUDA/container runtime and the official model
weights before FoundationPose itself can run. The Humble-side ROS bridge is in
lunar_rover_manipulation and does not depend on those GPU packages.
EOF
