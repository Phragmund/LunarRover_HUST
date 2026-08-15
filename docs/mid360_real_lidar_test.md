# Livox Mid-360 Real Hardware Smoke Test

This note is for first-power-on functional testing of a second-hand Livox
Mid-360 before integrating it with the rover stack.

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

This repository currently has:

```text
host IP in config:  192.168.1.5
lidar IP in config: 192.168.1.12
```

Before running the ROS driver, make sure the PC Ethernet IP matches
`host_net_info` in `src/lunar_rover_sensors/config/livox_mid360.json`, and make
sure `lidar_configs[0].ip` matches the actual Mid-360 IP.

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
`192.168.1.5/24` and make sure the lidar IP is `192.168.1.12`.

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

- Driver initializes successfully.
- A Livox point cloud topic is published.
- Topic frequency is near the configured publish frequency, usually 10 Hz.
- `point_num` is nonzero in `livox_ros_driver2/msg/CustomMsg`.

For RViz PointCloud2 testing, use `xfer_format:=0` and inspect the published
topic name from `ros2 topic list`.
