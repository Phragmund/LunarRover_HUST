#!/usr/bin/env bash
# Started only by the phone console's "开始建图导航" control.
set -o pipefail

repo_root=/home/phragmund/LunarRover
progress_file=/tmp/rover_mapping_startup_status
cd "$repo_root"
report_progress() {
  # Atomic replace avoids the phone reading a partially-written UTF-8 message.
  printf '%s\n' "$1" >"${progress_file}.tmp"
  mv "${progress_file}.tmp" "$progress_file"
  echo "[progress] $1"
}
exec 9>/tmp/start_real_mapping_navigation.lock
if ! flock -n 9; then
  echo "A mapping/navigation startup is already in progress; ignoring duplicate request."
  exit 0
fi
exec >>/tmp/start_real_mapping_navigation.log 2>&1

if pgrep -f '[f]astlio_mapping' >/dev/null; then
  echo "FAST-LIO process already exists; refusing a duplicate startup request."
  report_progress "启动失败：检测到已有 FAST-LIO 进程。"
  # Non-zero matters: the phone UI must not report a successful navigation
  # startup merely because an orphaned/half-started process was detected.
  exit 1
fi

echo "=== Phone requested mapping/navigation startup ==="
report_progress "步骤 1/5：正在启动 FAST-LIO 定位…"
source /opt/ros/humble/setup.bash
source "$repo_root/install/setup.bash"

setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source /home/phragmund/LunarRover/install/setup.bash && exec ros2 run lunar_rover_localization rear_sector_cloud_filter.py --ros-args -p input_topic:=/livox/lidar -p output_topic:=/livox/lidar_front190 -p base_frame:=base_link -p front_half_angle_deg:=95.0' </dev/null >/tmp/rear_sector_filter.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source /home/phragmund/LunarRover/install/setup.bash && exec ros2 run fast_lio fastlio_mapping --ros-args --params-file /home/phragmund/LunarRover/src/lunar_rover_localization/config/fast_lio2_mid360_real.yaml -p use_sim_time:=false -r /Odometry:=/localization/lio_odometry_raw -r /path:=/localization/lio_path_raw -r /cloud_registered:=/localization/cloud_registered_raw -r /cloud_registered_body:=/localization/cloud_registered_body -r /cloud_effected:=/localization/cloud_effected -r /Laser_map:=/localization/lio_map_cloud_raw' </dev/null >/tmp/fastlio.log 2>&1
# The rover's powered-but-stationary IMU has a measured gyro bias around
# 0.033 rad/s, so the nominal 0.02 threshold rejects every startup sample.
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source /home/phragmund/LunarRover/install/setup.bash && exec ros2 run lunar_rover_localization fast_lio2_map_frame_node.py --ros-args -r __node:=fast_lio2_map_frame --params-file /home/phragmund/LunarRover/src/lunar_rover_localization/config/fast_lio2_map_frame.yaml -p use_sim_time:=false -p imu_mount_pitch_source:=startup_imu_gravity -p imu_mount_pitch_max_gyro_radps:=0.06 -p lio_body_frame:=livox_frame -p level_correction_source:=startup' </dev/null >/tmp/mapframe_main.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source /home/phragmund/LunarRover/install/setup.bash && exec ros2 run lunar_rover_localization fast_lio2_map_frame_node.py --ros-args -r __node:=fast_lio2_map_frame_visualization --params-file /home/phragmund/LunarRover/src/lunar_rover_localization/config/fast_lio2_map_frame.yaml -p use_sim_time:=false -p imu_mount_pitch_source:=startup_imu_gravity -p imu_mount_pitch_max_gyro_radps:=0.06 -p lio_body_frame:=livox_frame -p level_correction_source:=startup' </dev/null >/tmp/mapframe_viz.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source /home/phragmund/LunarRover/install/setup.bash && exec python3 /home/phragmund/LunarRover/src/lunar_rover_navigation/scripts/lio_nav_tf_bridge.py' </dev/null >/tmp/lio_tf.log 2>&1
report_progress "步骤 2/5：正在校正 IMU 与等待 FAST-LIO 里程计…"
odom_ready=false
for i in $(seq 1 15); do
  if timeout 3 ros2 topic echo --once /localization/lio_odometry >/dev/null 2>&1; then
    echo "FAST-LIO odometry ready (round $i)"
    odom_ready=true
    break
  fi
  report_progress "步骤 2/5：等待 FAST-LIO 里程计（第 $i/15 轮）…"
  sleep 2
done
if [ "$odom_ready" != true ]; then
  report_progress "启动失败：未收到 FAST-LIO 里程计。"
  exit 1
fi

report_progress "步骤 3/5：正在启动地形建图与 Nav2…"
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source /home/phragmund/LunarRover/install/setup.bash && exec ros2 launch lunar_rover_mapping terrain_mapping.launch.py params_file:=/home/phragmund/LunarRover/src/lunar_rover_mapping/config/terrain_mapper_real.yaml use_sim_time:=false enable_cloud_fusion:=false enable_elevation_grid:=false' </dev/null >/tmp/terrain.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source /home/phragmund/LunarRover/install/setup.bash && exec ros2 launch lunar_rover_navigation navigation.launch.py params_file:=/home/phragmund/LunarRover/src/lunar_rover_navigation/config/nav2_fast_lio2_params_real.yaml use_sim_time:=false run_command_adapters:=false' </dev/null >/tmp/nav2.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source /home/phragmund/LunarRover/install/setup.bash && exec ros2 run lunar_rover_navigation twist_stamper_node --ros-args -r __node:=twist_stamper --params-file /home/phragmund/LunarRover/src/lunar_rover_navigation/config/twist_stamper_real.yaml' </dev/null >/tmp/twist.log 2>&1

report_progress "步骤 4/5：正在激活 Nav2 导航服务…"
nav2_ready=false
for i in $(seq 1 15); do
  if ros2 action list 2>/dev/null | grep -q '/compute_path_to_pose'; then
    echo "Nav2 action ready (round $i)"
    nav2_ready=true
    break
  fi
  report_progress "步骤 4/5：等待 Nav2 导航服务（第 $i/15 轮）…"
  sleep 2
done
if [ "$nav2_ready" != true ]; then
  report_progress "启动失败：Nav2 导航服务未就绪。"
  exit 1
fi
report_progress "步骤 5/5：正在启动探索与 RViz…"
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source /home/phragmund/LunarRover/install/setup.bash && exec ros2 launch lunar_rover_exploration frontier_exploration.launch.py params_file:=/home/phragmund/LunarRover/src/lunar_rover_exploration/config/frontier_explorer_real.yaml' </dev/null >/tmp/frontier.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source /home/phragmund/LunarRover/install/setup.bash && exec rviz2 -d /home/phragmund/LunarRover/src/lunar_rover_mapping/rviz/real_nav2.rviz --ros-args -p use_sim_time:=false' </dev/null >/tmp/rviz.log 2>&1
report_progress "已完成：建图、Nav2、探索与 RViz 已启动。"
echo "Mapping/navigation startup submitted."
