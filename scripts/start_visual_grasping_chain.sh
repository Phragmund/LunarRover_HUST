#!/usr/bin/env bash
# Start the real Gemini 335L and the visual-grasp state-machine chain.
# This script deliberately does not own the chassis, LiDAR, or Nav2 processes.
set -euo pipefail

repo_root=/home/phragmund/LunarRover
cd "$repo_root"
exec >>/tmp/start_visual_grasping_chain.log 2>&1

set +u
source /opt/ros/humble/setup.bash
source "$repo_root/install/setup.bash"
set -u

# Replace only stale vision-chain processes.  Bracketed patterns avoid matching
# this launcher itself.
for pattern in '[o]rbbec_camera' '[t]arget_detector_node.py' '[g]rasp_state_machine_node.py' '[m]ission_executive_node.py' '[a]ctive_search_node.py' '[g]rasp_target_localizer'; do
  pkill -TERM -f "$pattern" 2>/dev/null || true
done
sleep 1

setsid -f bash -lc "source /opt/ros/humble/setup.bash && source '$repo_root/install/setup.bash' && exec ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera config_file_path:='$repo_root/src/lunar_rover_sensors/config/gemini335_real.yaml'" \
  </dev/null >/tmp/orbbec_camera.log 2>&1

setsid -f bash -lc "source /opt/ros/humble/setup.bash && source '$repo_root/install/setup.bash' && exec ros2 launch lunar_rover_manipulation real_grasping.launch.py grasp_backend:=state_machine detector_mode:=fusion detector_backend:=ultralytics segmentation_model:='$repo_root/weights/best.pt' active_search:=true auto_trigger_grasp:=false periodic_sweep:=false mission_executive:=true" \
  </dev/null >/tmp/visual_grasping.log 2>&1

echo "Visual grasp chain submitted: Gemini camera + detector + localizer + grasp state machine + mission executive."
