#!/usr/bin/env bash
set -u

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
handoff_dir="${REAL_CHASSIS_HANDOFF_DIR:-$repo_root/tmp/3508_can_handoff}"
steering_port="${REAL_CHASSIS_STEERING_PORT:-/dev/ttyACM0}"
failures=0

pass() { echo "[PASS] $*"; }
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

if id -nG | tr ' ' '\n' | grep -qx input; then
  pass "user belongs to input group"
else
  fail "user does not belong to input group"
fi

for path in \
  "$handoff_dir/dmcan/__init__.py" \
  "$handoff_dir/dlls/libdm_device.so" \
  "$handoff_dir/dlls/libstdc++.so.6" \
  "$handoff_dir/dlls/libusb-1.0.so.0"; do
  if [[ -r "$path" ]]; then
    pass "readable: $path"
  else
    fail "missing or unreadable: $path"
  fi
done

export LD_LIBRARY_PATH="$handoff_dir/dlls:${LD_LIBRARY_PATH:-}"
if python3 -c 'import rclpy, serial, fastapi, uvicorn' 2>/dev/null; then
  pass "ROS, pyserial, FastAPI, and uvicorn imports"
else
  fail "Python runtime dependency import failed"
fi

if [[ -e "$steering_port" && -r "$steering_port" && -w "$steering_port" ]]; then
  pass "steering CDC port is readable/writable: $steering_port"
else
  fail "steering CDC port missing or inaccessible: $steering_port"
fi

if python3 - "$handoff_dir" <<'PY'
import sys

handoff = sys.argv[1]
sys.path.insert(0, handoff)
from dmcan import DmCanContext

ctx = DmCanContext()
try:
    count = ctx.find_devices()
    print("USB2CANFD devices:", count)
    raise SystemExit(0 if count > 0 else 1)
finally:
    ctx.destroy()
PY
then
  pass "USB2CANFD SDK can enumerate the adapter"
else
  fail "USB2CANFD adapter not found or SDK failed"
fi

if pgrep -af 'chassis_driver_node.py|chassis_cmd_mux_node.py|web_joystick_server.py' >/dev/null; then
  fail "another real-control process appears to be running"
else
  pass "no duplicate real-control process detected"
fi

if (( failures > 0 )); then
  echo "Preflight failed with $failures problem(s). Chassis was not started." >&2
  exit 1
fi

echo "Preflight passed. No motor command was sent."
