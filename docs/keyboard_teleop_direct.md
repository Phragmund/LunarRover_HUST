# Direct Keyboard Teleop

The keyboard controller now uses only the direct `/dev/input/event*` backend.
The old focused-terminal stdin backend was removed because terminal auto-repeat
can lag after a long key hold and cannot observe key release events reliably.

## Agent Startup Checklist

Use this exact sequence after a reboot or when handing the rover to another
agent:

```bash
cd /home/phragmund/LunarRover
source /opt/ros/humble/setup.bash
source install/setup.bash
id
```

The `id` output should include `input` on this laptop. If it does not, the
helper script can still apply a temporary ACL with `sudo setfacl`, but a
logout/reboot is required after adding the user to the `input` group.

Start the direct controller with the helper, not with `sudo ros2 run`:

```bash
cd /home/phragmund/LunarRover
scripts/run_direct_keyboard_teleop.sh
```

Success looks like this:

```text
Starting direct keyboard teleop on /keyboard/chassis_cmd_vel
Keyboard event ACL targets:
  /dev/input/event...
input backend: direct /dev/input
devices:
  /dev/input/...
mode=stop             v=+0.000 m/s wz=+0.000 rad/s
```

If the direct backend cannot read keyboard devices, it exits with an error.
Fix `/dev/input` permissions or device selection before driving; there is no
terminal fallback.

Do not use these commands for the real direct backend:

```bash
sudo ros2 run lunar_rover_navigation keyboard_rover_teleop.py
ros2 run lunar_rover_navigation keyboard_rover_teleop.py --ros-args -p input_backend:=terminal
```

The first runs ROS as root and can miss the user's ROS environment. The second
requests the removed terminal backend. Use the helper script so the workspace is
sourced, keyboard devices are discovered, and the node runs as the normal user.

## Quick Probe

Validate keyboard event delivery without ROS:

```bash
cd /home/phragmund/LunarRover
./tools/keyboard_event_probe.py --all-keys
```

Press `W/A/S/D` and left Shift. A working setup prints `press`, optional
`repeat`, and immediate `release` lines. Left Shift should show as
`left_shift` or `code=42` when `--all-keys` is used.

If this fails with permission errors, either run the helper once so it applies
the temporary ACL, or apply the ACL manually:

```bash
cd /home/phragmund/LunarRover
sudo setfacl -m "u:${USER}:r" /dev/input/event3 /dev/input/event6 /dev/input/event10
./tools/keyboard_event_probe.py --all-keys
```

Device numbers can change after reboot. Prefer the helper script when possible
because it discovers `/dev/input/by-id/*event-kbd` and
`/dev/input/by-path/*event-kbd` symlinks automatically.

## Isolated ROS Test

This publishes to `/keyboard_probe/cmd_vel`, not the chassis:

```bash
cd /home/phragmund/LunarRover
scripts/run_direct_keyboard_teleop.sh --probe
```

In another terminal:

```bash
cd /home/phragmund/LunarRover
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic hz /keyboard_probe/cmd_vel
```

Expected rate is about `20 Hz`.

To verify the command content, use:

```bash
ros2 topic echo /keyboard_probe/cmd_vel
```

Expected values:

```text
W       linear.x > 0, angular.z = 0
S       linear.x < 0, angular.z = 0
A       linear.x = 0, angular.z > 0
D       linear.x = 0, angular.z < 0
W+A     linear.x > 0, angular.z > 0
W+D     linear.x > 0, angular.z < 0
S+A     linear.x < 0, angular.z > 0
S+D     linear.x < 0, angular.z < 0
LeftShift+W/A/S/D combinations double the nonzero command values by default
SPACE   linear.x = 0, angular.z = 0
```

## Chassis Control

After the isolated test works, start the direct backend on the keyboard override topic:

```bash
cd /home/phragmund/LunarRover
scripts/run_direct_keyboard_teleop.sh
```

Controls:

```text
W/S       forward / reverse
A/D       in-place left / right rotation
W/S+A/D   Ackermann-style turning while driving
LeftShift 2x drive power while held
SPACE     stop
Ctrl-C    quit direct backend
```

This is equivalent to:

```bash
ros2 run lunar_rover_navigation keyboard_rover_teleop.py --ros-args \
  -p input_backend:=direct \
  -p command_topic:=/keyboard/chassis_cmd_vel
```

`chassis_cmd_mux` forwards `/keyboard/chassis_cmd_vel` to `/chassis/cmd_vel`
while keyboard input is active. If no keyboard command arrives for 5 seconds,
the mux returns chassis control to the navigation command stream.

The boost multiplier is configurable:

```bash
ros2 run lunar_rover_navigation keyboard_rover_teleop.py --ros-args \
  -p input_backend:=direct \
  -p command_topic:=/keyboard/chassis_cmd_vel \
  -p boost_multiplier:=2.0
```

Use the explicit form only after permissions are already correct. The helper is
safer because it sources the workspace, discovers event devices, and applies the
temporary read ACL before launching ROS as the normal user.

Do not run the ROS teleop node with `sudo`. The helper script only uses `sudo`
to apply a temporary read ACL to keyboard event devices, then launches ROS as
the normal user. The ACL usually disappears after reboot or device reconnect,
so run the helper again after a restart.

## Common Failures

`Package 'lunar_rover_navigation' not found`

Source the workspace first:

```bash
source /opt/ros/humble/setup.bash
source /home/phragmund/LunarRover/install/setup.bash
```

`Unable to read keyboard event devices` or `Permission denied`

Run the helper script without `--no-acl`, or confirm the user is in the `input`
group and has logged in again after that group change:

```bash
id
scripts/run_direct_keyboard_teleop.sh
```

`No keyboard event devices found`

List the kernel devices and pass the laptop keyboard events explicitly:

```bash
find /dev/input/by-id /dev/input/by-path -maxdepth 1 -type l -name '*event-kbd' -print
scripts/run_direct_keyboard_teleop.sh --device /dev/input/event3,/dev/input/event6,/dev/input/event10
```

No visible response while keys are pressed

Make sure the physical laptop keyboard is being used. The direct backend reads
kernel keyboard events and does not depend on terminal focus, but it can only
see devices that exist on this machine and are readable by the current user.

## Known Good Laptop Devices

On the tested laptop, these were the relevant keyboard event targets:

```text
/dev/input/event3
/dev/input/event6
/dev/input/event10
```

The helper script discovers equivalent `/dev/input/by-id/*event-kbd` and
`/dev/input/by-path/*event-kbd` symlinks automatically.

## Policy

For this project, all future keyboard driving should use:

```bash
scripts/run_direct_keyboard_teleop.sh
```

Do not reintroduce or use focused-terminal stdin teleop for rover driving.
