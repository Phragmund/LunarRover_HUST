#!/usr/bin/env bash
set -euo pipefail

if [[ "$(lsb_release -sc)" != "jammy" ]]; then
  echo "This bootstrap targets Ubuntu 22.04 (jammy)." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y curl gnupg2 lsb-release locales software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo add-apt-repository -y universe
ROS_KEY_TMP="$(mktemp)"
curl -fsSL --connect-timeout 20 --retry 3 \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o "${ROS_KEY_TMP}"
sudo install -m 0644 "${ROS_KEY_TMP}" /usr/share/keyrings/ros-archive-keyring.gpg
rm -f "${ROS_KEY_TMP}"
ROS_APT_URL="${ROS_APT_URL:-https://mirrors.aliyun.com/ros2/ubuntu}"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_APT_URL} $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt-get update
sudo apt-get install -y \
  ros-humble-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-pytest \
  python3-vcstool \
  python3-rosdep \
  ros-humble-ros-gz \
  ros-humble-gz-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-depth-image-proc \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  nlohmann-json3-dev

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro humble

echo
echo "ROS 2 Humble base dependencies installed."
echo "Run: source /opt/ros/humble/setup.bash"
echo "Then: ./scripts/import_dependencies.sh && ./scripts/import_hardware_dependencies.sh && ./scripts/build.sh"
