#!/usr/bin/env bash
# Gemini 335L 功能自检 / 环境检查
#
# 用法:
#   ./scripts/gemini335_check.sh           # 静态检查 + USB 检测（不启动相机）
#   ./scripts/gemini335_check.sh --live    # 额外启动驱动，实测话题与帧率

set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PASS=0; FAIL=0; WARN=0
ok()   { printf "  [PASS] %s\n" "$1"; PASS=$((PASS+1)); }
bad()  { printf "  [FAIL] %s\n" "$1"; FAIL=$((FAIL+1)); }
warn() { printf "  [WARN] %s\n" "$1"; WARN=$((WARN+1)); }

echo "=================================================="
echo " Gemini 335L 功能自检"
echo "=================================================="

echo "== 1. ROS 2 环境 =="
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  ok "ROS 2 Humble 已 source"
else
  bad "未找到 /opt/ros/humble/setup.bash"
fi
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
  ok "工作区 install/setup.bash 已 source"
else
  bad "工作区未构建（缺 install/setup.bash），请先运行 ./scripts/build.sh"
fi

echo "== 2. 驱动包 =="
if ros2 pkg list 2>/dev/null | grep -qx orbbec_camera; then
  ok "orbbec_camera 已安装"
else
  bad "orbbec_camera 未安装"
fi
if ros2 pkg list 2>/dev/null | grep -qx lunar_rover_sensors; then
  ok "lunar_rover_sensors 已安装"
else
  bad "lunar_rover_sensors 未安装"
fi

echo "== 3. RViz =="
if command -v rviz2 >/dev/null 2>&1; then
  ok "rviz2 可用"
else
  bad "rviz2 未安装"
fi

echo "== 4. Orbbec udev 规则 =="
if [[ -f /etc/udev/rules.d/99-obsensor-libusb.rules ]]; then
  ok "/etc/udev/rules.d/99-obsensor-libusb.rules 已安装"
else
  bad "udev 规则未安装，非 root 用户将无法访问相机"
  echo "       安装命令:"
  echo "       sudo cp $ROOT_DIR/install/orbbec_camera/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger"
fi

echo "== 5. video 组 (仅 v4l2 后端需要) =="
if groups | grep -qw video; then
  ok "当前用户在 video 组"
else
  warn "当前用户不在 video 组（默认 libuvc 后端不依赖此组，可忽略）"
fi

echo "== 6. USB 设备 =="
if lsusb | grep -qi "2bc5"; then
  ok "检测到 Orbbec 设备（VID 2bc5）"
  lsusb | grep -i "2bc5"
  if lsusb | grep -qi "2bc5:0804"; then
    ok "识别为 Gemini 335L（PID 0804）"
  else
    warn "PID 不是 0804，请确认是否为 Gemini 335L"
  fi
else
  bad "未检测到 Orbbec 设备（VID 2bc5）。请接入相机后重试"
fi

if [[ "${1:-}" == "--live" ]]; then
  echo
  echo "== 7. 实机流验证 =="
  echo "    后台启动驱动 (start_rviz:=false) ..."
  ros2 launch lunar_rover_sensors gemini335_test.launch.py start_rviz:=false > /tmp/gemini335_live.log 2>&1 &
  LPID=$!
  echo "    等待设备初始化 15s ..."
  sleep 15
  echo "--- /camera 相关话题 ---"
  ros2 topic list 2>/dev/null | grep -E "^/camera/" || echo "    (无 /camera 话题，设备可能未初始化)"
  for T in /camera/color/image_raw /camera/depth/image_raw /camera/depth/points; do
    echo "--- 帧率: $T (采样 3s) ---"
    timeout 3 ros2 topic hz "$T" 2>&1 | tail -n 4 || echo "    (无数据)"
  done
  echo "    停止驱动 ..."
  kill "$LPID" 2>/dev/null
  pkill -f "camera_container" 2>/dev/null
  echo "    驱动日志: /tmp/gemini335_live.log"
fi

echo
echo "=================================================="
echo " 结果: PASS=$PASS  FAIL=$FAIL  WARN=$WARN"
echo "=================================================="

if [[ "$FAIL" -gt 0 ]]; then
  echo "存在 FAIL 项，请先解决后再进行开箱测试。"
  exit 1
else
  echo "环境检查通过，可运行 ./scripts/gemini335_test.sh 进行开箱测试。"
  exit 0
fi
