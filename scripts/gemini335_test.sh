#!/usr/bin/env bash
# Gemini 335L 开箱即测：启动相机驱动 + RViz（RGB 彩图 / 深度灰度图 / 深度点云）
#
# 用法:
#   ./scripts/gemini335_test.sh               # 相机 + RViz
#   ./scripts/gemini335_test.sh --no-rviz     # 只启动相机驱动（不弹 RViz）
#
# 前置条件（一次性）:
#   1) 已安装 Orbbec udev 规则（见 scripts/gemini335_check.sh 或 README）
#   2) 相机通过 USB3 线接入

set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

START_RVIZ="true"
if [[ "${1:-}" == "--no-rviz" ]]; then
  START_RVIZ="false"
fi

echo "==> Sourcing ROS 2 Humble + workspace"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${ROOT_DIR}/install/setup.bash"

echo
echo "==> USB 设备检查 (Orbbec VID 2bc5)"
if lsusb | grep -qi "2bc5"; then
  lsusb | grep -i "2bc5"
  echo "    [OK] 检测到 Orbbec 相机"
else
  echo "    [警告] 未检测到 Orbbec 相机（VID 2bc5）。请确认相机已通过 USB3 线连接。"
fi

echo
echo "==> udev 规则检查"
if [[ -f /etc/udev/rules.d/99-obsensor-libusb.rules ]]; then
  echo "    [OK] /etc/udev/rules.d/99-obsensor-libusb.rules 已安装"
else
  echo "    [警告] 未找到 udev 规则，非 root 用户可能无法访问相机。"
  echo "           请执行: sudo cp ${ROOT_DIR}/install/orbbec_camera/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger"
fi

echo
if [[ "${START_RVIZ}" == "true" ]]; then
  echo "==> 启动 Gemini 335L 相机 + RViz (Ctrl+C 退出)"
else
  echo "==> 启动 Gemini 335L 相机 (Ctrl+C 退出)"
fi
echo "    话题:"
echo "      RGB 彩图   : /camera/color/image_raw"
echo "      深度灰度图 : /camera/depth/image_raw"
echo "      深度点云   : /camera/depth/points"
echo

exec ros2 launch lunar_rover_sensors gemini335_test.launch.py start_rviz:=${START_RVIZ}
