#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_direct_keyboard_teleop.sh [--probe] [--topic TOPIC] [--device DEVICE[,DEVICE...]] [--no-acl]

Defaults:
  --topic /keyboard/chassis_cmd_vel

Options:
  --probe              Publish to /keyboard_probe/cmd_vel instead of the chassis.
  --topic TOPIC        Publish TwistStamped commands to TOPIC.
  --device DEVICES     Comma-separated /dev/input event devices. If omitted,
                       keyboard_rover_teleop.py auto-discovers keyboard devices.
  --no-acl             Do not run setfacl before launching.

This script should be run as the normal desktop user, not with sudo. It uses
sudo only for the temporary /dev/input read ACL, then starts ROS normally.
EOF
}

topic="/keyboard/chassis_cmd_vel"
devices=""
apply_acl=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --probe)
      topic="/keyboard_probe/cmd_vel"
      shift
      ;;
    --topic)
      topic="${2:-}"
      shift 2
      ;;
    --device|--devices)
      devices="${2:-}"
      shift 2
      ;;
    --no-acl)
      apply_acl=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$(id -u)" -eq 0 ]]; then
  echo "Run this as the normal user, not with sudo." >&2
  exit 2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if [[ -f install/setup.bash ]]; then
  # shellcheck source=/dev/null
  set +u
  source install/setup.bash
  set -u
else
  echo "Missing install/setup.bash. Build/source the workspace first." >&2
  exit 2
fi

acl_targets=()
if [[ -n "$devices" ]]; then
  IFS=',' read -r -a requested_devices <<< "$devices"
  for device in "${requested_devices[@]}"; do
    [[ -n "$device" ]] || continue
    if [[ -e "$device" ]]; then
      acl_targets+=("$(readlink -f "$device")")
    else
      echo "Input device does not exist: $device" >&2
      exit 2
    fi
  done
else
  while IFS= read -r device; do
    [[ -n "$device" ]] || continue
    acl_targets+=("$(readlink -f "$device")")
  done < <(find /dev/input/by-id /dev/input/by-path -maxdepth 1 -type l -name '*event-kbd' 2>/dev/null | sort)
fi

if [[ "${#acl_targets[@]}" -eq 0 ]]; then
  echo "No keyboard event devices found under /dev/input/by-id or /dev/input/by-path." >&2
  exit 2
fi

if [[ "$apply_acl" -eq 1 ]]; then
  sudo setfacl -m "u:${USER}:r" "${acl_targets[@]}"
fi

ros_args=(
  run lunar_rover_navigation keyboard_rover_teleop.py
  --ros-args
  -p input_backend:=direct
  -p command_topic:="$topic"
)

if [[ -n "$devices" ]]; then
  ros_args+=(-p input_device:="$devices")
fi

echo "Starting direct keyboard teleop on $topic"
echo "Keyboard event ACL targets:"
printf '  %s\n' "${acl_targets[@]}"
exec ros2 "${ros_args[@]}"
