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

visual_pid_file=/tmp/lunar_rover_visual_launch.pid
camera_pid_file=/tmp/lunar_rover_camera_launch.pid
watchdog_pid_file=/tmp/lunar_rover_visual_watchdog.pid

stop_recorded_group() {
  local pid_file="$1"
  if [[ -f "$pid_file" ]]; then
    local pid
    pid=$(<"$pid_file")
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
      # Each launch below owns a dedicated session, so this only stops its
      # own ROS children instead of relying on broad process-name matches.
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
    rm -f "$pid_file"
  fi
}

# Replace only stale vision-chain processes.  Bracketed patterns avoid matching
# this launcher itself.
for pattern in '[o]rbbec_camera' '[c]amera_container' '[t]arget_detector_node.py' '[t]arget_fusion_detector_node.py' '[g]rasp_state_machine_node.py' '[m]ission_executive_node.py' '[a]ctive_search_node.py' '[g]rasp_target_localizer'; do
  pkill -TERM -f "$pattern" 2>/dev/null || true
done
stop_recorded_group "$camera_pid_file"
stop_recorded_group "$visual_pid_file"
sleep 1

start_camera() {
  setsid bash -lc "source /opt/ros/humble/setup.bash && source '$repo_root/install/setup.bash' && exec ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera config_file_path:='$repo_root/src/lunar_rover_sensors/config/gemini335_real.yaml'" \
    </dev/null >/tmp/orbbec_camera.log 2>&1 &
  echo $! >"$camera_pid_file"
}

start_visual_launch() {
  setsid bash -lc "source /opt/ros/humble/setup.bash && source '$repo_root/install/setup.bash' && exec ros2 launch lunar_rover_manipulation real_grasping.launch.py grasp_backend:=state_machine detector_mode:=fusion detector_backend:=ultralytics segmentation_model:='$repo_root/weights/best.pt' active_search:=true auto_trigger_grasp:=false periodic_sweep:=false mission_executive:=true" \
    </dev/null >/tmp/visual_grasping.log 2>&1 &
  echo $! >"$visual_pid_file"
}

start_camera
# A connected Gemini can occasionally expose its topics while its UVC stream is
# stalled.  Do not start a blind visual chain: verify one RGB frame and retry
# the driver once before declaring the chain ready.
if ! timeout 10 ros2 topic echo /camera/color/image_raw --qos-reliability best_effort --once >/dev/null 2>&1; then
  pkill -TERM -f '[o]rbbec_camera|[c]amera_container' 2>/dev/null || true
  sleep 2
  start_camera
  timeout 12 ros2 topic echo /camera/color/image_raw --qos-reliability best_effort --once >/dev/null 2>&1
fi

bash "$repo_root/scripts/restart_handeye_tf.sh"

start_visual_launch

# The console consumes this exact overlay topic.  Treat a launch with no
# overlay frame as failed, clean only the vision processes, and retry once.
if ! timeout 18 ros2 topic echo /perception/debug/fusion_overlay/compressed --qos-reliability best_effort --once >/dev/null 2>&1; then
  echo "Vision overlay missing after first start; restarting visual nodes once."
  for pattern in '[t]arget_detector_node.py' '[t]arget_fusion_detector_node.py' '[g]rasp_state_machine_node.py' '[m]ission_executive_node.py' '[a]ctive_search_node.py' '[g]rasp_target_localizer'; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
  done
  sleep 2
  stop_recorded_group "$visual_pid_file"
  start_visual_launch
  timeout 18 ros2 topic echo /perception/debug/fusion_overlay/compressed --qos-reliability best_effort --once >/dev/null 2>&1
fi

# Keep the UI's image source alive after this one-shot starter has returned.
# The watchdog restarts the affected session only after two consecutive failed
# frame checks; a transient DDS discovery delay therefore cannot flap hardware.
if [[ -f "$watchdog_pid_file" ]]; then
  old_watchdog=$(<"$watchdog_pid_file")
  if [[ "$old_watchdog" =~ ^[0-9]+$ ]] && kill -0 "$old_watchdog" 2>/dev/null; then
    kill -TERM "$old_watchdog" 2>/dev/null || true
  fi
fi
setsid bash "$repo_root/scripts/visual_grasping_watchdog.sh" </dev/null >/tmp/visual_grasping_watchdog.log 2>&1 &
echo $! >"$watchdog_pid_file"

echo "Visual grasp chain ready: Gemini camera + fusion overlay + localizer + grasp state machine + mission executive."
