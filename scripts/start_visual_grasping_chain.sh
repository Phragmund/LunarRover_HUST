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

# Disarm the previous watchdog before touching anything it supervises.  The
# watchdog restarts the visual session on its own once the overlay has been
# missing for two polls, so a launcher that killed the nodes while an old
# watchdog was still running got its session resurrected underneath it -- two
# `real_grasping.launch.py` trees, both writing the same pid file, both
# publishing the same node names.  Ownership has to be serialised: this script
# holds the chain while it works and hands the watchdog back a healthy one.
stop_watchdog() {
  if [[ -f "$watchdog_pid_file" ]]; then
    local pid
    pid=$(<"$watchdog_pid_file")
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
    rm -f "$watchdog_pid_file"
  fi
  # The pid file can be stale or missing after a manual start, so also match
  # the script by name.  The bracket keeps this from matching our own command.
  pkill -TERM -f '[v]isual_grasping_watchdog.sh' 2>/dev/null || true
  # Wait for it to actually exit: it restarts nodes on a 5 s poll, so starting
  # the teardown while it is still alive reopens the same race.
  for _ in {1..20}; do
    pgrep -f '[v]isual_grasping_watchdog.sh' >/dev/null 2>&1 || break
    sleep 0.5
  done
}

stop_watchdog

# Replace only stale vision-chain processes.  Bracketed patterns avoid matching
# this launcher itself.  The launch wrappers are matched too, so a tree that
# lost its pid file (an earlier race, or a manual start) is still cleaned up
# instead of lingering as a second publisher of every node name.
for pattern in '[o]rbbec_camera' '[c]amera_container' '[r]eal_grasping.launch.py' '[t]arget_detector_node.py' '[t]arget_fusion_detector_node.py' '[g]rasp_state_machine_node.py' '[m]ission_executive_node.py' '[a]ctive_search_node.py' '[g]rasp_target_localizer'; do
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
bash "$repo_root/scripts/restart_handeye_tf.sh"

start_visual_launch

# The console consumes this exact overlay topic.  Treat a launch with no
# overlay frame as failed, clean only the vision processes, and retry once.
# Loading the self-trained ultralytics checkpoint dominates this wait, so the
# window is wider than the RGB one.  Guarded for the same `set -e` reason as
# the RGB probe above.
overlay_frame_available() {
  timeout "$1" ros2 topic echo /perception/debug/fusion_overlay/compressed \
    --qos-reliability best_effort --once >/dev/null 2>&1
}

if ! overlay_frame_available 90; then
  echo "Vision overlay missing after first start; restarting camera and visual nodes once."
  for pattern in '[t]arget_detector_node.py' '[t]arget_fusion_detector_node.py' '[g]rasp_state_machine_node.py' '[m]ission_executive_node.py' '[a]ctive_search_node.py' '[g]rasp_target_localizer'; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
  done
  stop_recorded_group "$visual_pid_file"
  stop_recorded_group "$camera_pid_file"
  pkill -TERM -f '[o]rbbec_camera|[c]amera_container' 2>/dev/null || true
  sleep 5
  start_camera
  bash "$repo_root/scripts/restart_handeye_tf.sh"
  start_visual_launch
  if ! overlay_frame_available 90; then
    echo "ERROR: no /perception/debug/fusion_overlay/compressed frame after a restart." >&2
    echo "ERROR: the console would show no camera picture; see /tmp/visual_grasping.log and /tmp/orbbec_camera.log" >&2
    exit 1
  fi
fi

# Keep the UI's image source alive after this one-shot starter has returned.
# The watchdog restarts the affected session only after two consecutive failed
# frame checks; a transient DDS discovery delay therefore cannot flap hardware.
# The previous watchdog was already stopped at the top of this script, before
# the chain was touched, so nothing can race this handover.
setsid bash "$repo_root/scripts/visual_grasping_watchdog.sh" </dev/null >/tmp/visual_grasping_watchdog.log 2>&1 &
echo $! >"$watchdog_pid_file"

echo "Visual grasp chain ready: Gemini camera + fusion overlay + localizer + grasp state machine + mission executive."
