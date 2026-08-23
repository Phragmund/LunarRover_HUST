#!/usr/bin/env bash
set -u

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
h7_port="${REAL_CHASSIS_H7_PORT:-${REAL_CHASSIS_STEERING_PORT:-/dev/ttyACM0}}"
failures=0

pass() { echo "[PASS] $*"; }
warn() { echo "[WARN] $*" >&2; }
fail() { echo "[FAIL] $*" >&2; failures=$((failures + 1)); }

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  fail "missing /opt/ros/humble/setup.bash"
fi
if [[ -f "$repo_root/install/setup.bash" ]]; then
  set +u
  source "$repo_root/install/setup.bash"
  set -u
else
  fail "workspace is not built: missing install/setup.bash"
fi

if python3 -c 'import rclpy, serial, fastapi, uvicorn; import lunar_rover_msgs.msg' 2>/dev/null; then
  pass "ROS, pyserial, FastAPI, uvicorn, and lunar_rover_msgs imports"
else
  fail "Python runtime dependency import failed"
fi

if [[ -e "$h7_port" && -r "$h7_port" && -w "$h7_port" ]]; then
  pass "H7 CDC port is readable/writable: $h7_port"
else
  fail "H7 CDC port missing or inaccessible: $h7_port"
fi

if id -nG | tr ' ' '\n' | grep -qx input; then
  pass "user belongs to input group"
else
  warn "user does not belong to input group; phone control still works, keyboard_controller may need root/input permission"
fi

if pgrep -af 'motion_solver.py|keyboard_controller.py|web_joystick_server.py|chassis_driver_node.py|chassis_cmd_mux_node.py' >/dev/null; then
  fail "another real-control process appears to be running"
else
  pass "no duplicate real-control process detected"
fi

if (( failures > 0 )); then
  echo "Preflight failed with $failures problem(s). Chassis was not started." >&2
  exit 1
fi

echo "Preflight passed. No motor command was sent."
