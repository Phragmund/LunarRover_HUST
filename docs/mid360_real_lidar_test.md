# Livox Mid-360 Real Hardware Smoke Test

This note is for first-power-on functional testing of a second-hand Livox
Mid-360 before integrating it with the rover stack.

## Tested Working Configuration (verified 2026-08)

This exact unit was tested and confirmed working on this machine. Its network
settings differ from the factory defaults described below: it is a second-hand
unit that a previous owner reconfigured onto a different subnet, so **do not**
assume the factory-default IP applies.

| Item | Value |
| --- | --- |
| Lidar IP | `192.168.2.204` |
| Lidar serial number | last two digits `96` (factory default would be `192.168.1.196`, but this unit has been moved to `192.168.2.x`) |
| PC Ethernet | `enp7s0` = `192.168.2.50/24` (NetworkManager connection `livox-mid360`, autoconnect enabled) |
| Driver config | `src/lunar_rover_sensors/config/livox_mid360.json` already holds host `192.168.2.50` / lidar `192.168.2.204` |
| Driver | `livox_ros_driver2` v1.2.6, already built in this workspace |

Verified results with `xfer_format:=0` (PointCloud2):

- `/livox/lidar`: `sensor_msgs/msg/PointCloud2`, ~20000 points/frame @ 10 Hz,
  fields `x y z intensity tag line timestamp` (LivoxPointXyzitl).
- `/livox/imu`: `sensor_msgs/msg/Imu` @ 200 Hz.
- RViz displays `/livox/lidar` directly using
  `debug/mid360_smoke.rviz` (Fixed Frame `livox_frame`).

For FAST-LIO2, use `src/lunar_rover_localization/config/fast_lio2_mid360_real.yaml`
(`lidar_type: 4` = MID360 PointCloud2 handler, `use_sim_time: false`).

### If the lidar IP is unknown (re-discovery)

If the unit stops answering at `192.168.2.204` (e.g. a different unit, or it was
reset to factory defaults), discover its actual IP instead of guessing:

1. Connect Ethernet + power, then configure the PC to `192.168.2.50/24` first
   (covers this unit) and also try `192.168.1.50/24` (covers factory-default
   units).
2. Open the official `LivoxViewer2` (see `docs/livox_mid360_vendor_index.md`) and
   refresh adapters/devices — it broadcasts a Livox `LidarSearch` and reports the
   device IP and serial number regardless of the configured subnet.
3. Alternatively, a UDP ping-sweep of `192.168.1.0/24` and `192.168.2.0/24`
   (`ping -c1 -W1 192.168.X.Y`) finds the address if the unit still responds to
   ARP.

Once the real IP is known, update `lidar_configs[0].ip` in
`src/lunar_rover_sensors/config/livox_mid360.json` and put the PC on the matching
subnet.

## Official Files Prepared

The official binaries/manuals are not committed to this repository.  See
[`docs/livox_mid360_vendor_index.md`](livox_mid360_vendor_index.md) for the
current file list and official download links.

## Hardware Safety Checklist

Use the official Livox Aviation Connector 1-to-3 Splitter Cable for first tests
if available. It splits the Mid-360 M12 connector into power, RJ-45 Ethernet,
and function/sync wires.

- Power input: DC 9-27 V. Use 12 V for bench testing.
- Power budget: normal working power is about 6.5 W, startup peak can be much higher. Use a 12 V supply capable of at least 2 A for margin.
- Polarity: red or positive wire to `+12 V`, black or negative wire to `GND`.
- Ethernet: connect the splitter cable RJ-45 directly to the PC Ethernet port.
- Do not connect RJ-45 to any PoE injector, PoE switch, or PoE port.
- Do not connect or disconnect bare power wires while the supply is live.
- Keep other active lidars from pointing directly into the Mid-360 aperture.
- During bench tests, place the unit on a non-conductive surface with airflow.

If you only have the bare M12 connector, stop and check the pinout in the user
manual before wiring. Do not guess pin numbers from wire colors unless the cable
vendor documents them.

## Network Facts

Mid-360 uses static IPv4. Factory default device IP is:

```text
192.168.1.1XX
```

where `XX` is the last two digits of the Mid-360 serial number.

The PC must be on the same subnet, for example:

```text
PC Ethernet IP: 192.168.1.50/24
Subnet mask:    255.255.255.0
Gateway:        optional, normally 192.168.1.1
```

This repository is currently configured for the tested unit:

```text
host IP in config:  192.168.2.50
lidar IP in config: 192.168.2.204
```

Before running the ROS driver, make sure the PC Ethernet IP matches
`host_net_info` in `src/lunar_rover_sensors/config/livox_mid360.json`, and make
sure `lidar_configs[0].ip` matches the actual Mid-360 IP. (For a different unit,
follow the re-discovery steps at the top of this document.)

## Current PC Environment

Checked on this machine:

- Ubuntu 22.04.5 LTS.
- ROS 2 Humble available after `source /opt/ros/humble/setup.bash`.
- RViz2 available after sourcing ROS.
- `livox_ros_driver2` is already built in this workspace.
- Driver version prints as `Livox Ros Driver2 Version: 1.2.6`.
- Ethernet interface seen locally: `enp7s0`.
- Current `enp7s0` state before hardware connection: down/unavailable.

## First Test Procedure

The steps below describe a factory-default unit on `192.168.1.x`. For the unit
tested in this repository, use the values in "Tested Working Configuration" above
(`192.168.2.50` on the PC, `192.168.2.204` on the lidar) instead of `192.168.1.50`.

1. Read the serial number on the Mid-360 body and compute the expected IP:

```text
SN last two digits = XX
expected lidar IP  = 192.168.1.1XX
```

2. Power off the bench power supply.

3. Connect the M12 splitter cable to the Mid-360 and tighten the lock nut so
there is no visible gap.

4. Connect RJ-45 from the splitter cable directly to the PC Ethernet port.

5. Connect power wires to the bench supply: positive to `+12 V`, negative to
`GND`.

6. Turn on the supply and check current draw. If current is abnormal, voltage
drops heavily, the supply current-limits, there is smell/heat/noise, or wiring
looks wrong, power off immediately.

7. Configure the PC Ethernet IP. For the official quick-start style test:

```bash
sudo ip addr flush dev enp7s0
sudo ip addr add 192.168.1.50/24 dev enp7s0
sudo ip link set enp7s0 up
ip -br addr show enp7s0
```

8. Check whether packets are visible:

```bash
sudo tcpdump -ni enp7s0 udp and net 192.168.1.0/24
```

9. Download and unpack LivoxViewer2 for Ubuntu using the links in
`docs/livox_mid360_vendor_index.md`, then start it from the unpacked directory:

```bash
cd /home/phragmund/LunarRover
'<your-livox-viewer2-dir>/LivoxViewer2.sh'
```

10. In Viewer 2, refresh adapters/devices. The adapter should be the Ethernet
interface on `192.168.1.x`, and the device should appear with its IP and serial
number. First verify live point cloud display before any firmware upgrade.

## ROS Driver Smoke Test

Use this after Viewer 2 has found the device and the actual IP is known.

If using the current repo config unchanged, set the PC Ethernet IP to
`192.168.2.50/24` and make sure the lidar IP is `192.168.2.204`.

```bash
cd /home/phragmund/LunarRover
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p user_config_path:=/home/phragmund/LunarRover/src/lunar_rover_sensors/config/livox_mid360.json
```

In another terminal:

```bash
cd /home/phragmund/LunarRover
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list | grep livox
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once
```

Expected minimum result:

- Driver initializes successfully and logs `livox/lidar publish use PointCloud2 format`.
- `/livox/lidar` is published at about the configured frequency (10 Hz).
- `/livox/imu` is published at 200 Hz.
- One `/livox/lidar` message contains ~20000 points (`width * height > 0`).

With `xfer_format:=0` the point cloud is `sensor_msgs/msg/PointCloud2`
(LivoxPointXyzitl fields) and can be displayed directly in RViz. With
`xfer_format:=1` it becomes `livox_ros_driver2/msg/CustomMsg`, which is what the
AVIA (`lidar_type: 1`) FAST-LIO2 input expects; the real-unit FAST-LIO2 config in
this repo uses PointCloud2 with `lidar_type: 4` instead.
