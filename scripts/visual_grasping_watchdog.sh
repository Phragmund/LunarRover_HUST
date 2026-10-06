#!/usr/bin/env bash
# Supervise the two image-producing stages used by the web console.  Camera
# hardware and visual inference are restarted independently so a detector
# shutdown never needlessly tears down a healthy Gemini stream.
repo_root=/home/phragmund/LunarRover
camera_pid_file=/tmp/lunar_rover_camera_launch.pid
visual_pid_file=/tmp/lunar_rover_visual_launch.pid
handeye_pid_file=/tmp/lunar_rover_handeye_preview.pid

# ROS's generated setup files reference optional variables.  Do not enable
# nounset until they have been sourced.
set +u
source /opt/ros/humble/setup.bash
source "$repo_root/install/setup.bash"
set -u

stop_group() {
  local pid_file="$1"
  [[ -f "$pid_file" ]] || return 0
  local pid
  pid=$(<"$pid_file")
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  fi
  rm -f "$pid_file"
}

start_camera() {
  setsid bash -lc "source /opt/ros/humble/setup.bash && source '$repo_root/install/setup.bash' && exec ros2 launch orbbec_camera gemini_330_series.launch.py camera_name:=camera config_file_path:='$repo_root/src/lunar_rover_sensors/config/gemini335_real.yaml'" </dev/null >/tmp/orbbec_camera.log 2>&1 &
  echo $! >"$camera_pid_file"
}

start_visual() {
  setsid bash -lc "source /opt/ros/humble/setup.bash && source '$repo_root/install/setup.bash' && exec ros2 launch lunar_rover_manipulation real_grasping.launch.py grasp_backend:=state_machine detector_mode:=fusion detector_backend:=ultralytics segmentation_model:='$repo_root/weights/best.pt' active_search:=true auto_trigger_grasp:=false periodic_sweep:=false mission_executive:=true" </dev/null >/tmp/visual_grasping.log 2>&1 &
  echo $! >"$visual_pid_file"
}

ensure_handeye_tf() {
  if pgrep -f '[h]andeye_tf_preview_node.py' >/dev/null 2>&1; then
    return 0
  fi
  echo "$(date -Is) dynamic hand-eye TF publisher absent; starting it"
  bash "$repo_root/scripts/restart_handeye_tf.sh" --ensure
}

camera_failures=0
overlay_failures=0
while true; do
  sleep 5
  ensure_handeye_tf
  if timeout 4 ros2 topic echo /camera/color/image_raw --qos-reliability best_effort --once >/dev/null 2>&1; then
    camera_failures=0
  else
    camera_failures=$((camera_failures + 1))
  fi

  if (( camera_failures >= 2 )); then
    echo "$(date -Is) camera stream absent twice; restarting camera and visual sessions"
    stop_group "$visual_pid_file"
    stop_group "$camera_pid_file"
    pkill -TERM -f '[o]rbbec_camera|[c]amera_container' 2>/dev/null || true
    sleep 2
    start_camera
    sleep 8
    start_visual
    camera_failures=0
    overlay_failures=0
    continue
  fi

  if timeout 4 ros2 topic echo /perception/debug/fusion_overlay/compressed --qos-reliability best_effort --once >/dev/null 2>&1; then
    overlay_failures=0
  else
    overlay_failures=$((overlay_failures + 1))
  fi
  if (( overlay_failures >= 2 )); then
    echo "$(date -Is) fusion overlay absent twice; restarting visual session only"
    stop_group "$visual_pid_file"
    pkill -TERM -f '[t]arget_detector_node.py|[t]arget_fusion_detector_node.py|[g]rasp_state_machine_node.py|[m]ission_executive_node.py|[a]ctive_search_node.py|[g]rasp_target_localizer' 2>/dev/null || true
    sleep 2
    start_visual
    overlay_failures=0
  fi
done
