#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

handoff_dir="${REAL_CHASSIS_HANDOFF_DIR:-$repo_root/tmp/3508_can_handoff}"
steering_port="${REAL_CHASSIS_STEERING_PORT:-/dev/ttyACM0}"
web_port="${WEB_JOYSTICK_PORT:-8000}"
web_token="${WEB_JOYSTICK_TOKEN:-}"
if [[ -z "$web_token" ]]; then
  web_token="$(python3 -c 'import secrets; print(secrets.token_hex(12))')"
fi

REAL_CHASSIS_HANDOFF_DIR="$handoff_dir" \
REAL_CHASSIS_STEERING_PORT="$steering_port" \
  scripts/check_real_chassis.sh

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export LD_LIBRARY_PATH="$handoff_dir/dlls:${LD_LIBRARY_PATH:-}"

ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
if [[ -n "$ip" ]]; then
  echo "Phone controller: http://$ip:$web_port/?token=$web_token"
fi
echo "The chassis starts DISABLED. Enable only after lifting the wheels or clearing the area:"
echo "  ros2 topic pub --once /chassis/controller_enabled std_msgs/msg/Bool '{data: true}'"

exec ros2 launch lunar_rover_bringup real_chassis.launch.py \
  handoff_dir:="$handoff_dir" \
  steering_port:="$steering_port" \
  web_port:="$web_port" \
  web_access_token:="$web_token" \
  "$@"
