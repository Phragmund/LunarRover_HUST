#!/usr/bin/env bash
# Restart only the hand-eye TF publisher; never touch camera or arm control.
set -eo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
source /opt/ros/humble/setup.bash
source "$repo_root/install/setup.bash"
exec 9>/tmp/lunar_rover_handeye_tf.lock
flock -w 15 9

case "${1:-}" in
  ""|--ensure) ;;
  *) echo "Usage: bash scripts/restart_handeye_tf.sh [--ensure]" >&2; exit 2 ;;
esac

# Validate before stopping the existing publisher. Match only this repository's
# Python hand-eye tool, rather than broadly killing ROS or camera processes.
set +e
python3 - "${1:-}" <<'PY'
import os
from pathlib import Path
import signal
import sys
import time

sys.path.insert(0, str(Path.cwd() / "tools"))
from handeye_calibration_registry import load_registry

_, selected_id, _ = load_registry()
pids = []
for entry in Path("/proc").iterdir():
    if not entry.name.isdigit():
        continue
    try:
        args = (entry / "cmdline").read_bytes().split(b"\0")
        cwd = (entry / "cwd").resolve()
        is_python = args and Path(os.fsdecode(args[0])).name.startswith("python")
        tool_args = [Path(os.fsdecode(arg)) for arg in args[1:] if arg]
        is_tool = any(arg.name == "handeye_tf_preview_node.py" and
                      (arg if arg.is_absolute() else cwd / arg).resolve() ==
                      Path.cwd() / "tools/handeye_tf_preview_node.py" for arg in tool_args)
        if is_python and is_tool:
            pids.append(int(entry.name))
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
if sys.argv[1] == "--ensure" and pids:
    sys.exit(10)
for pid in pids:
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
deadline = time.monotonic() + 6
while any(Path(f"/proc/{pid}").exists() for pid in pids):
    if time.monotonic() >= deadline:
        raise RuntimeError("hand-eye publisher did not stop; refusing a duplicate publisher")
    time.sleep(0.1)
print(f"Starting hand-eye TF profile: {selected_id}")
PY
handeye_status=$?
set -e
if [[ "$handeye_status" == 10 ]]; then
  exit 0
elif [[ "$handeye_status" != 0 ]]; then
  exit "$handeye_status"
fi

setsid bash -c 'source /opt/ros/humble/setup.bash; source install/setup.bash; exec python3 tools/handeye_tf_preview_node.py' \
  </dev/null >/tmp/handeye_tf_preview.log 2>&1 9>&- &
pid=$!
printf '%s\n' "$pid" >/tmp/lunar_rover_handeye_preview.pid
sleep 1
if ! kill -0 "$pid" 2>/dev/null; then
  echo "Hand-eye TF failed to start; see /tmp/handeye_tf_preview.log" >&2
  exit 1
fi
