#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

h7_port="${REAL_CHASSIS_H7_PORT:-${REAL_CHASSIS_STEERING_PORT:-/dev/ttyACM0}}"
web_port="${WEB_JOYSTICK_PORT:-8000}"
web_token="${WEB_JOYSTICK_TOKEN:-}"
if [[ -z "$web_token" ]]; then
  web_token="$(python3 -c 'import secrets; print(secrets.token_hex(12))')"
fi

REAL_CHASSIS_H7_PORT="$h7_port" scripts/check_real_chassis.sh

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ip="$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -Ev '^(192\.168\.2\.50|127\.)' | head -n1)"
if [[ -z "$ip" ]]; then
  ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
fi
if [[ -n "$ip" ]]; then
  echo "Phone controller: http://$ip:$web_port/?token=$web_token"
fi
echo "Using H7 port: $h7_port"
echo "WARNING: motion_solver enables the 3508 drive on startup. Keep wheels lifted or the area clear."

exec ros2 launch "$repo_root/src/lunar_rover_bringup/launch/real_chassis.launch.py" \
  h7_port:="$h7_port" \
  web_port:="$web_port" \
  web_access_token:="$web_token" \
  "$@"
