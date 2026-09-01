#!/usr/bin/env bash
# 实车纯雷达 Nav2 全链路干净重启脚本
set -o pipefail
cd /home/phragmund/LunarRover

echo "=== [1/5] 停止旧进程 ==="
pkill -TERM -f 'fastlio_mapping' 2>/dev/null || true
pkill -TERM -f 'livox_ros_driver2' 2>/dev/null || true
pkill -TERM -f 'robot_state_publisher' 2>/dev/null || true
pkill -TERM -f 'fast_lio2_map_frame' 2>/dev/null || true
pkill -TERM -f 'lio_nav_tf_bridge' 2>/dev/null || true
pkill -TERM -f 'nav_to_manual_mux' 2>/dev/null || true
pkill -TERM -f 'terrain_mapper' 2>/dev/null || true
pkill -TERM -f 'terrain_mapping.launch' 2>/dev/null || true
pkill -TERM -f 'twist_stamper' 2>/dev/null || true
pkill -TERM -f 'web_joystick_server' 2>/dev/null || true
pkill -TERM -f 'motion_solver' 2>/dev/null || true
pkill -TERM -f 'frontier_explorer' 2>/dev/null || true
pkill -TERM -f 'navigation.launch' 2>/dev/null || true
pkill -TERM -x controller_server 2>/dev/null || true
pkill -TERM -x planner_server 2>/dev/null || true
pkill -TERM -x bt_navigator 2>/dev/null || true
pkill -TERM -x behavior_server 2>/dev/null || true
pkill -TERM -x smoother_server 2>/dev/null || true
pkill -TERM -x velocity_smoother 2>/dev/null || true
pkill -TERM -x lifecycle_manager 2>/dev/null || true
pkill -TERM -x waypoint_follower 2>/dev/null || true
pkill -TERM -x rviz2 2>/dev/null || true
sleep 3

echo "=== [2/5] 生成 URDF 与 RSP 参数 ==="
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
xacro /home/phragmund/LunarRover/install/lunar_rover_description/share/lunar_rover_description/urdf/lunar_rover.urdf.xacro use_sim:=false > /tmp/lunar_rover_real.urdf
python3 -c "import yaml; urdf=open('/tmp/lunar_rover_real.urdf').read(); yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':urdf,'use_sim_time':False}}}, open('/tmp/rsp_params.yaml','w'), allow_unicode=True, sort_keys=False)"

echo "=== [3/5] 启动传感器+SLAM+TF ==="
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame -p user_config_path:=/home/phragmund/LunarRover/src/lunar_rover_sensors/config/livox_mid360.json' </dev/null >/tmp/lidar.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file /tmp/rsp_params.yaml' </dev/null >/tmp/rsp.log 2>&1
sleep 3
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run fast_lio fastlio_mapping --ros-args --params-file /home/phragmund/LunarRover/src/lunar_rover_localization/config/fast_lio2_mid360_real.yaml -p use_sim_time:=false -r /Odometry:=/localization/lio_odometry_raw -r /path:=/localization/lio_path_raw -r /cloud_registered:=/localization/cloud_registered_raw -r /cloud_registered_body:=/localization/cloud_registered_body -r /cloud_effected:=/localization/cloud_effected -r /Laser_map:=/localization/lio_map_cloud_raw' </dev/null >/tmp/fastlio.log 2>&1
sleep 2
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run lunar_rover_localization fast_lio2_map_frame_node.py --ros-args -r __node:=fast_lio2_map_frame --params-file /home/phragmund/LunarRover/src/lunar_rover_localization/config/fast_lio2_map_frame.yaml -p use_sim_time:=false -p imu_mount_pitch_source:=startup_imu_gravity -p lio_body_frame:=livox_frame -p level_correction_source:=startup' </dev/null >/tmp/mapframe_main.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run lunar_rover_localization fast_lio2_map_frame_node.py --ros-args -r __node:=fast_lio2_map_frame_visualization --params-file /home/phragmund/LunarRover/src/lunar_rover_localization/config/fast_lio2_map_frame.yaml -p use_sim_time:=false -p imu_mount_pitch_source:=startup_imu_gravity -p lio_body_frame:=livox_frame -p level_correction_source:=startup' </dev/null >/tmp/mapframe_viz.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 /home/phragmund/LunarRover/src/lunar_rover_navigation/scripts/lio_nav_tf_bridge.py' </dev/null >/tmp/lio_tf.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 /home/phragmund/LunarRover/src/lunar_rover_navigation/scripts/nav_to_manual_mux.py' </dev/null >/tmp/mux.log 2>&1

echo "=== [4/5] 等待 FAST-LIO 里程计, 再启动建图+导航+遥控 ==="
for i in $(seq 1 45); do
  if timeout 3 ros2 topic echo --once /localization/lio_odometry >/dev/null 2>&1; then
    echo "FAST-LIO 里程计就绪 (round $i)"
    break
  fi
  sleep 2
done
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 launch lunar_rover_mapping terrain_mapping.launch.py params_file:=/home/phragmund/LunarRover/src/lunar_rover_mapping/config/terrain_mapper_real.yaml use_sim_time:=false enable_cloud_fusion:=false enable_elevation_grid:=false' </dev/null >/tmp/terrain.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 launch lunar_rover_navigation navigation.launch.py params_file:=/home/phragmund/LunarRover/src/lunar_rover_navigation/config/nav2_fast_lio2_params_real.yaml use_sim_time:=false run_command_adapters:=false' </dev/null >/tmp/nav2.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run lunar_rover_navigation twist_stamper_node --ros-args -r __node:=twist_stamper --params-file /home/phragmund/LunarRover/src/lunar_rover_navigation/config/twist_stamper_real.yaml' </dev/null >/tmp/twist.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run lunar_rover_navigation web_joystick_server.py --ros-args -p port:=8000 -p access_token:=rover2026 -p command_topic:=/web/manual_control -p use_sim_time:=false' </dev/null >/tmp/web.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run lunar_rover_control motion_solver.py --ros-args -p h7_port:=/dev/ttyACM0 -p use_sim_time:=false' </dev/null >/tmp/motion.log 2>&1

echo "=== [5/5] 等待 Nav2 action 就绪, 再启动 frontier 与 RViz ==="
for i in $(seq 1 45); do
  if ros2 action list 2>/dev/null | grep -q '/compute_path_to_pose'; then
    echo "Nav2 /compute_path_to_pose 就绪 (round $i)"
    break
  fi
  sleep 2
done
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 launch lunar_rover_exploration frontier_exploration.launch.py params_file:=/home/phragmund/LunarRover/src/lunar_rover_exploration/config/frontier_explorer_real.yaml' </dev/null >/tmp/frontier.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec rviz2 -d /home/phragmund/LunarRover/src/lunar_rover_mapping/rviz/real_nav2.rviz --ros-args -p use_sim_time:=false' </dev/null >/tmp/rviz.log 2>&1

echo "done."
