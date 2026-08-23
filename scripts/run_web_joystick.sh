#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if [[ -f install/setup.bash ]]; then
  set +u
  source install/setup.bash
  set -u
else
  echo "Missing install/setup.bash. Build the workspace first (scripts/build.sh)." >&2
  exit 2
fi

port="${WEB_JOYSTICK_PORT:-8000}"
token="${WEB_JOYSTICK_TOKEN:-}"
if [[ -z "$token" ]]; then
  token="$(python3 -c 'import secrets; print(secrets.token_hex(12))')"
fi

ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "Web joystick server on port $port (override with WEB_JOYSTICK_PORT=NNNN)."
if [[ -n "$ip" ]]; then
  echo "Phone/laptop URL: http://$ip:$port/?token=$token"
fi

exec ros2 run lunar_rover_navigation web_joystick_server.py --ros-args \
  -p port:="$port" \
  -p access_token:="$token" \
  "$@"
