#!/usr/bin/env bash
# 实车纯雷达 Nav2 全链路干净重启脚本
set -o pipefail
cd /home/phragmund/LunarRover

echo "=== [1/5] 停止旧进程 ==="
pkill -TERM -f 'fastlio_mapping' 2>/dev/null || true
pkill -TERM -f 'livox_ros_driver2' 2>/dev/null || true
pkill -TERM -f 'robot_state_publisher' 2>/dev/null || true
pkill -TERM -f 'fast_lio2_map_frame' 2>/dev/null || true
pkill -TERM -f 'rear_sector_cloud_filter' 2>/dev/null || true
pkill -TERM -f 'lio_nav_tf_bridge' 2>/dev/null || true
pkill -TERM -f 'nav_to_manual_mux' 2>/dev/null || true
pkill -TERM -f 'terrain_mapper' 2>/dev/null || true
pkill -TERM -f 'terrain_mapping.launch' 2>/dev/null || true
pkill -TERM -f 'twist_stamper' 2>/dev/null || true
pkill -TERM -f 'web_joystick_server' 2>/dev/null || true
pkill -TERM -f 'motion_solver' 2>/dev/null || true
pkill -TERM -f 'frontier_explorer' 2>/dev/null || true
pkill -TERM -f 'navigation.launch' 2>/dev/null || true
# Linux task names are capped at 15 characters (for example
# controller_server becomes controller_serv), so pkill -x silently misses
# several Nav2 executables.  Match each fully resolved executable instead.
pkill -TERM -f '/opt/ros/humble/lib/nav2_controller/controller_server' 2>/dev/null || true
pkill -TERM -f '/opt/ros/humble/lib/nav2_planner/planner_server' 2>/dev/null || true
pkill -TERM -f '/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator' 2>/dev/null || true
pkill -TERM -f '/opt/ros/humble/lib/nav2_behaviors/behavior_server' 2>/dev/null || true
pkill -TERM -f '/opt/ros/humble/lib/nav2_smoother/smoother_server' 2>/dev/null || true
pkill -TERM -f '/opt/ros/humble/lib/nav2_velocity_smoother/velocity_smoother' 2>/dev/null || true
pkill -TERM -f '/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager' 2>/dev/null || true
pkill -TERM -f '/opt/ros/humble/lib/nav2_waypoint_follower/waypoint_follower' 2>/dev/null || true
pkill -TERM -x rviz2 2>/dev/null || true
sleep 3

echo "=== [2/5] 生成 URDF 与 RSP 参数 ==="
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
xacro /home/phragmund/LunarRover/install/lunar_rover_description/share/lunar_rover_description/urdf/lunar_rover.urdf.xacro use_sim:=false external_camera_tf:=true > /tmp/lunar_rover_real.urdf
python3 -c "import yaml; urdf=open('/tmp/lunar_rover_real.urdf').read(); yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':urdf,'use_sim_time':False}}}, open('/tmp/rsp_params.yaml','w'), allow_unicode=True, sort_keys=False)"

echo "=== [3/3] 启动待命传感器、手机遥控与底盘 ==="
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame -p user_config_path:=/home/phragmund/LunarRover/src/lunar_rover_sensors/config/livox_mid360.json' </dev/null >/tmp/lidar.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file /tmp/rsp_params.yaml' </dev/null >/tmp/rsp.log 2>&1
# Keep this bridge live in standby mode: the phone joystick publishes
# /web/manual_control, while motion_solver intentionally accepts only the
# arbitrated /manual_control topic.  Nav2 inputs simply remain absent until the
# phone requests mapping/navigation startup.
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec python3 /home/phragmund/LunarRover/src/lunar_rover_navigation/scripts/nav_to_manual_mux.py --ros-args --params-file /home/phragmund/LunarRover/src/lunar_rover_navigation/config/twist_stamper_real.yaml' </dev/null >/tmp/mux.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run lunar_rover_navigation web_joystick_server.py --ros-args -p port:=8000 -p access_token:=rover2026 -p command_topic:=/web/manual_control -p use_sim_time:=false' </dev/null >/tmp/web.log 2>&1
setsid -f bash -lc 'source /opt/ros/humble/setup.bash && source install/setup.bash && exec ros2 run lunar_rover_control motion_solver.py --ros-args -p h7_port:=/dev/ttyACM0 -p use_sim_time:=false' </dev/null >/tmp/motion.log 2>&1

echo "待命完成：手机网页已启动。请在手机端点击『开始建图导航』后才会启动 FAST-LIO、地形建图、Nav2 和探索。"
