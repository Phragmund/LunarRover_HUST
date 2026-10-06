#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${PORT:-8099}"
LOG_FILE="${LOG_FILE:-/tmp/ros_web_console.log}"
CONSOLE_PATH="${CONSOLE_PATH:-$ROOT_DIR/src/lunar_rover_hardware/ubuntu_upper_dev_handoff_20260820/tools/web/rover_web_controller.html}"
RAW_TOPIC="${RAW_TOPIC:-/h7/raw_command}"
RAW_FEEDBACK_TOPIC="${RAW_FEEDBACK_TOPIC:-/h7/rx_frame}"
CAMERA_TOPIC="${CAMERA_TOPIC:-/perception/debug/fusion_overlay/compressed}"
MAX_CAMERA_HZ="${MAX_CAMERA_HZ:-5.0}"
ACCESS_TOKEN="${ACCESS_TOKEN:-}"
START_MOTION_SOLVER="${START_MOTION_SOLVER:-0}"
H7_PORT="${H7_PORT:-/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_357635623333-if00}"
MOTION_LOG_FILE="${MOTION_LOG_FILE:-/tmp/motion_solver_web_console.log}"
ENABLE_DRIVE_ON_START="${ENABLE_DRIVE_ON_START:-false}"

stop_matching_python() {
  local pattern="$1"
  mapfile -t pids < <(
    ps -eo pid=,args= |
      awk -v pat="$pattern" '$0 ~ pat && $0 !~ /awk / {print $1}'
  )
  if ((${#pids[@]} > 0)); then
    kill "${pids[@]}" 2>/dev/null || true
    sleep 0.5
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        kill -9 "$pid" 2>/dev/null || true
      fi
    done
  fi
}

echo "[ros-web-console] stopping old ros_web_console_server.py instances..."
stop_matching_python 'python3 .*ros_web_console_server\.py'

if [[ "$START_MOTION_SOLVER" == "1" ]]; then
  echo "[ros-web-console] stopping old motion_solver.py instances..."
  stop_matching_python 'python3 .*motion_solver\.py'
fi

cd "$ROOT_DIR"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

args=(
  --ros-args
  -p "port:=$PORT"
  -p "console_path:=$CONSOLE_PATH"
  -p "raw_command_topic:=$RAW_TOPIC"
  -p "raw_feedback_topic:=$RAW_FEEDBACK_TOPIC"
  -p "camera_topic:=$CAMERA_TOPIC"
  -p "max_camera_hz:=$MAX_CAMERA_HZ"
)

if [[ -n "$ACCESS_TOKEN" ]]; then
  args+=(-p "access_token:=$ACCESS_TOKEN")
fi

if [[ "$START_MOTION_SOLVER" == "1" ]]; then
  if [[ ! -e "$H7_PORT" ]]; then
    echo "[ros-web-console] H7 port does not exist: $H7_PORT" >&2
    exit 1
  fi
  if fuser "$H7_PORT" >/dev/null 2>&1; then
    echo "[ros-web-console] H7 port is busy: $H7_PORT" >&2
    echo "[ros-web-console] close Web Serial in Chrome or stop the old owner first." >&2
    fuser -v "$H7_PORT" >&2 || true
    exit 1
  fi
  echo "[ros-web-console] starting motion_solver on $H7_PORT ..."
  setsid -f python3 "$ROOT_DIR/src/lunar_rover_control/scripts/motion_solver.py" \
    --ros-args \
    -p "h7_port:=$H7_PORT" \
    -p "raw_command_topic:=$RAW_TOPIC" \
    -p "raw_feedback_topic:=$RAW_FEEDBACK_TOPIC" \
    -p "enable_drive_on_start:=$ENABLE_DRIVE_ON_START" \
    -p "use_sim_time:=false" \
    >"$MOTION_LOG_FILE" 2>&1 < /dev/null
fi

echo "[ros-web-console] starting on port $PORT ..."
setsid -f python3 "$ROOT_DIR/src/lunar_rover_control/scripts/ros_web_console_server.py" \
  "${args[@]}" >"$LOG_FILE" 2>&1 < /dev/null

for _ in {1..10}; do
  if ss -ltnp | grep -q ":$PORT "; then
    break
  fi
  sleep 0.5
done
if ss -ltnp | grep -q ":$PORT "; then
  echo "[ros-web-console] started"
  echo "[ros-web-console] local: http://127.0.0.1:$PORT"
  lan_ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  if [[ -n "${lan_ip:-}" ]]; then
    echo "[ros-web-console] lan:   http://$lan_ip:$PORT"
  fi
  echo "[ros-web-console] log:   $LOG_FILE"
  if [[ "$START_MOTION_SOLVER" == "1" ]]; then
    echo "[ros-web-console] h7:    $H7_PORT via motion_solver"
    echo "[ros-web-console] drive enable on start: $ENABLE_DRIVE_ON_START"
    echo "[ros-web-console] motion log: $MOTION_LOG_FILE"
  else
    echo "[ros-web-console] ROS command route needs motion_solver:"
    echo "                  START_MOTION_SOLVER=1 $0"
  fi
else
  echo "[ros-web-console] failed to listen on :$PORT; recent log:" >&2
  tail -80 "$LOG_FILE" >&2 || true
  exit 1
fi
