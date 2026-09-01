#!/usr/bin/env bash
# 手机遥控一键启动：motion_solver(拥有 H7 串口) + web_joystick_server(手机 Web 遥控)
#
# 用法:
#   scripts/run_phone_remote.sh                      # 自动探测 H7 串口
#   H7_PORT=/dev/ttyACM1 scripts/run_phone_remote.sh
#   WEB_JOYSTICK_PORT=9000 scripts/run_phone_remote.sh
#   WEB_JOYSTICK_TOKEN=xxx scripts/run_phone_remote.sh     # 默认无 token
#   MANUAL_CONTROL_TOPIC=/xxx scripts/run_phone_remote.sh  # 两个节点同步切换话题
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# ---------- 可覆盖参数 ----------
h7_port="${H7_PORT:-${REAL_CHASSIS_H7_PORT:-${REAL_CHASSIS_STEERING_PORT:-}}}"
web_port="${WEB_JOYSTICK_PORT:-8000}"
web_token="${WEB_JOYSTICK_TOKEN:-}"              # 空 = 无需 token
topic="${MANUAL_CONTROL_TOPIC:-/manual_control}"

# ---------- 停掉残留实例，避免串口/端口冲突 ----------
for name in motion_solver.py web_joystick_server.py; do
  if pgrep -f "$name" >/dev/null 2>&1; then
    echo "[清理] 停止残留进程: $name"
    pkill -f "$name" 2>/dev/null || true
  fi
done
sleep 1

# ---------- 加载 ROS 环境 ----------
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u; source /opt/ros/humble/setup.bash; set -u
else
  echo "错误: 未找到 /opt/ros/humble/setup.bash" >&2; exit 2
fi
if [[ -f install/setup.bash ]]; then
  set +u; source install/setup.bash; set -u
else
  echo "错误: 工作空间未构建 (缺少 install/setup.bash)，先运行 scripts/build.sh" >&2; exit 2
fi

# ---------- 探测 H7 USB CDC 串口 ----------
if [[ -z "$h7_port" ]]; then
  h7_port="$(ls /dev/serial/by-id/*STM32*Virtual* 2>/dev/null | head -n1)"
  if [[ -z "$h7_port" ]]; then
    h7_port="$(ls /dev/ttyACM* 2>/dev/null | head -n1)"
  fi
fi
if [[ -z "$h7_port" || ! -e "$h7_port" ]]; then
  echo "错误: 找不到 H7 USB CDC 串口 (检查 /dev/ttyACM*)。" >&2
  echo "  请插入下位机后重试，或手动指定: H7_PORT=/dev/ttyACM0 $0" >&2
  exit 2
fi

# ---------- 手机访问地址 ----------
ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
suffix=""
if [[ -n "$web_token" ]]; then suffix="?token=$web_token"; fi

echo "======================================================"
echo " 手机遥控启动"
echo "   H7 串口  : $h7_port"
echo "   话题     : $topic  (motion_solver 与 Web 端一致)"
echo "   Web 端口 : $web_port"
echo "   手机地址 : http://$ip:$web_port/$suffix"
echo "======================================================"
echo "警告: motion_solver 启动即下发 M3508 ENABLE 命令，请确认车轮架空/周围安全。"
echo "按 Ctrl+C 停止全部。"
echo

# ---------- 退出时收尾 ----------
solver_pid=""
web_pid=""
_cleaned=0
cleanup() {
  if [[ $_cleaned -ne 0 ]]; then return; fi
  _cleaned=1
  echo
  echo "停止节点..."
  if [[ -n "$solver_pid" ]]; then kill "$solver_pid" 2>/dev/null || true; fi
  if [[ -n "$web_pid" ]]; then kill "$web_pid" 2>/dev/null || true; fi
  sleep 1
  pkill -f "motion_solver.py" 2>/dev/null || true
  pkill -f "web_joystick_server.py" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ---------- 启动两个节点 ----------
ros2 run lunar_rover_control motion_solver.py --ros-args -p h7_port:="$h7_port" -p input_topic:="$topic" -p use_sim_time:=false &
solver_pid=$!

ros2 run lunar_rover_navigation web_joystick_server.py --ros-args -p port:="$web_port" -p access_token:="$web_token" -p command_topic:="$topic" -p use_sim_time:=false &
web_pid=$!

wait -n "$solver_pid" "$web_pid"
