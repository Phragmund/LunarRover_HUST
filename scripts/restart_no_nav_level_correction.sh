#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "[$(date +%T)] starting sim only, no nav"
ros2 launch lunar_rover_bringup bringup.launch.py \
  mode:=sim \
  world:=reachable_maze \
  gui:=true \
  headless:=false \
  rviz:=false \
  sensor_noise:=false \
  spawn_x:=-3.92 \
  spawn_y:=0.0 \
  spawn_z:=0.20 \
  spawn_yaw:=0.0 &

echo "[$(date +%T)] waiting 45s before FAST-LIO startup"
sleep 45

echo "[$(date +%T)] starting composite odometry"
ros2 run lunar_rover_mapping composite_wheel_imu_odometry_node.py \
  --ros-args \
  -r __node:=composite_wheel_imu_odometry \
  --params-file install/lunar_rover_mapping/share/lunar_rover_mapping/config/composite_odometry.yaml &

sleep 3

echo "[$(date +%T)] starting FAST-LIO localization with fixed map leveling"
ros2 launch lunar_rover_localization fast_lio2_localization.launch.py \
  run_fast_lio2:=true \
  direct_fast_lio_node:=true \
  run_tf_bridge:=true \
  run_map_frame_node:=true \
  run_input_normalizer:=false \
  run_sim_imu_adapter:=false \
  imu_mount_pitch_source:=fixed \
  imu_mount_pitch_deg:=35.273 \
  level_correction_source:=fixed \
  level_correction_roll_deg:=-0.58 \
  level_correction_pitch_deg:=-0.67 \
  level_correction_yaw_deg:=0.007 &

sleep 8

echo "[$(date +%T)] starting terrain/world markers/RViz"
ros2 launch lunar_rover_mapping terrain_mapping.launch.py \
  params_file:=install/lunar_rover_mapping/share/lunar_rover_mapping/config/terrain_mapper.yaml &

ros2 run lunar_rover_simulation world_model_marker_node.py \
  --ros-args \
  -p use_sim_time:=true \
  -p world_name:=reachable_maze \
  -p frame_id:=map \
  -p marker_topic:=/simulation/world_model_markers \
  -p alpha:=0.30 \
  -p outline_alpha:=0.30 \
  -p outline_width:=0.025 \
  -p map_origin_world_x:=-3.92 \
  -p map_origin_world_y:=0.0 \
  -p map_origin_world_z:=0.20 \
  -p map_origin_world_yaw:=0.0 &

rviz2 -d install/lunar_rover_mapping/share/lunar_rover_mapping/rviz/mapping.rviz \
  --ros-args \
  -p use_sim_time:=true &

wait
