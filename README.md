# 六轮摇臂月球车 ROS 2/Gazebo 基础研究环境

本仓库当前包含六轮摇臂月球车 ROS 2/Gazebo 仿真基础环境，以及基于 FAST-LIO2、地形建图、Nav2、前沿探索和键盘接管的在线建图探索链路。

## 当前保留内容

- ROS 2 Humble与Gazebo Fortress仿真；
- 标准地球重力 `9.80665 m/s²`；
- 六轮摇臂/副摇臂整车模型；
- 四轮转向、六轮驱动和摇臂差动控制接口；
- `/chassis/cmd_vel` 上位机底盘指令接口；
- Livox Mid-360原始点云、LaserScan和IMU话题；
- Gemini 335原始彩色图、深度图和相机内参；
- 深度图到点云的基础几何转换，仅用于数据观察；
- Gazebo和RViz基础可视化；
- 平地、台阶、岩石、混合地形、迷宫和图像洞穴测试世界；
- 实车Livox和Orbbec驱动启动入口。

## 历史说明

本仓库曾经清理为不包含 SLAM 或自主导航算法的基础环境。一阶段算法实验已保存在 Git 标签：

```text
backup-before-clean-slam-2026-07-27
```

当前工作区已经重新接入定位、建图、导航和探索模块。下面的门控启动链路是当前验证过的主入口。

## 项目定位

这是一个面向月球车熔岩洞自主探索任务的 ROS 2 Humble 工作区。仓库同时覆盖：

- Gazebo Fortress 仿真、机器人模型、传感器桥接和基础控制；
- Livox Mid-360 与 Gemini 335 的仿真/实车原始传感器入口；
- 轮速 + IMU 组合里程计、FAST-LIO2 适配和 `map -> odom -> base_link` TF 链路；
- FAST-LIO2 注册点云到 2.5D 地形代价图的建图模块；
- Nav2 地形代价层、速度适配、键盘接管和前沿探索；
- 面向联调的启动脚本、校验工具、标定工具和专项说明文档。

项目边界是“月球车底盘、传感器、定位建图导航探索链路”。机械臂、目标识别、采样任务状态机等仍应作为后续独立包接入，不应混入基础控制、仿真桥或模型包。

## 系统架构

当前主链路按传感器、定位、地图、导航和控制逐层解耦：

```text
Gazebo / 实车传感器
  -> /livox/points, /livox/imu, /livox/scan, /camera/*
  -> FAST-LIO2 + fast_lio2_map_frame_node.py
  -> /localization/lio_odometry, /localization/cloud_registered
  -> terrain_mapper_node.py
  -> /terrain/traversability
  -> Nav2 + lunar_rover_navigation/TerrainLayer
  -> /cmd_vel
  -> twist_stamper_node
  -> /navigation/chassis_cmd_vel
  -> chassis_cmd_mux_node.py
  -> /chassis/cmd_vel
  -> rover_controller + rocker_differential_controller
```

局部里程计链路并行运行，用于提供稳定的 `odom -> base_link`：

```text
/joint_states + /livox/imu
  -> composite_wheel_imu_odometry_node.py
  -> /localization/composite_odometry
  -> map_odom_tf_bridge.py
  -> map -> odom
```

仿真真值 `/simulation/ground_truth/odometry` 只用于评估和标定，不进入定位、建图、导航或探索闭环。

### ROS 包分层

| 包 | 职责 | 主要内容 |
| --- | --- | --- |
| `lunar_rover_msgs` | 公共接口 | `ChassisState.msg` 下位机状态消息 |
| `lunar_rover_description` | 机器人模型 | URDF/Xacro、RViz 模型视图 |
| `lunar_rover_simulation` | Gazebo 仿真 | 世界、模型、ROS-Gazebo bridge、Mid-360 仿真驱动、world marker |
| `lunar_rover_sensors` | 原始传感器入口 | 仿真深度点云转换、实车 Livox/Orbbec 启动 |
| `lunar_rover_control` | 底盘基础控制 | 4WS6WD 控制器、摇臂差动控制器、仿真底盘状态、轮反馈里程计 |
| `lunar_rover_localization` | FAST-LIO2 适配 | 输入归一化、仿真 IMU 适配、map 帧校正、`map->odom` 桥 |
| `lunar_rover_mapping` | 建图与里程计实验 | 组合轮速 IMU 里程计、2.5D terrain mapper、旧 2D/OctoMap 调试入口 |
| `lunar_rover_navigation` | 导航与控制适配 | Nav2 参数、地形代价层、TwistStamped 适配、键盘控制、命令 mux |
| `lunar_rover_exploration` | 自主探索 | 基于 traversability costmap 的前沿目标选择与 Nav2 action 调度 |
| `lunar_rover_bringup` | 集成启动 | 基础仿真/实车 bringup 和 FAST-LIO2 探索导航调试 launch |

更细的包内说明见 [`src/README.md`](src/README.md) 和各包 README。

## 文件架构

```text
.
├── README.md                         # 当前总览、构建和主启动说明
├── Makefile                          # validate/test/build 快捷目标
├── dependencies/                     # vcstool 固定依赖清单
├── scripts/                          # 环境安装、依赖导入、构建和联调启动脚本
├── tools/                            # 静态校验、键盘事件探测、里程计标定工具
├── docs/                             # 专项设计、联调、赛题资料和评估数据
├── debug/                            # 临时调试/生成数据
├── third_party/                      # 外部源码依赖导入位置
└── src/                              # ROS 2 工作区源码包
```

`build/`、`install/`、`log/` 和 `.deps/` 是本机构建输出或本地依赖安装目录，不是项目源码入口。

## 文档索引

- [`docs/fast_lio2_exploration_navigation.md`](docs/fast_lio2_exploration_navigation.md)：FAST-LIO2 + terrain mapping + Nav2 + exploration 链路说明。
- [`docs/composite_wheel_imu_odometry.md`](docs/composite_wheel_imu_odometry.md)：组合轮速 IMU 里程计设计和标定方法。
- [`docs/keyboard_teleop_direct.md`](docs/keyboard_teleop_direct.md)：直接读取 `/dev/input` 的键盘接管说明。
- [`docs/lower_controller_contract.md`](docs/lower_controller_contract.md)：上位机与实车下位机 ROS 接口契约。
- [`docs/simulation_worlds.md`](docs/simulation_worlds.md)：Gazebo 世界清单和 `image_cave` 地图生成说明。
- [`docs/SLAM_NAVIGATION_GUIDE.md`](docs/SLAM_NAVIGATION_GUIDE.md)：面向比赛任务的 SLAM、导航、采样总体技术路线。
- [`docs/project_info/`](docs/project_info/)：赛题资料、答疑材料和往届参考。

## 基础话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/chassis/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 上位机到底盘的运动指令 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 车轮、转向和悬架关节状态 |
| `/chassis/state` | `lunar_rover_msgs/msg/ChassisState` | 仿真下位机状态 |
| `/livox/points` | `sensor_msgs/msg/PointCloud2` | Gazebo直接输出的原始雷达点云 |
| `/livox/scan` | `sensor_msgs/msg/LaserScan` | Gazebo雷达扫描 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Mid-360原始IMU |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | Gemini 335彩色图 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Gemini 335原始深度图 |
| `/camera/depth_registered/points` | `sensor_msgs/msg/PointCloud2` | 由深度图几何转换得到的观察点云 |
| `/simulation/ground_truth/odometry` | `nav_msgs/msg/Odometry` | 仅用于仿真评估的真值，不是定位输出 |

## 构建

```bash
cd /home/phragmund/LunarRover
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 推荐启动链路：延迟传感器 + FAST-LIO2建图 + RViz + 键盘直控

这是当前验证通过的推荐链路。默认会启动 Gazebo、RViz、FAST-LIO2、地形建图和键盘控制，但不会启动 Nav2、frontier exploration 或 `chassis_cmd_mux`。键盘直接发布到底盘 `/chassis/cmd_vel`，不会有自动导航节点在 FAST-LIO 初始化或地图观察阶段移动车辆。

当前链路的关键变化是：Gazebo 先只启动基础仿真和基础 bridge，等待 `/clock`、`/simulation/ground_truth/odometry` 和额外稳定时间后，才启动 Gazebo 传感器 bridge、Mid-360 点云/IMU ROS 驱动和 FAST-LIO2。这可以避免复杂地图刚加载时的传感器数据提前进入 FAST-LIO。

核心原则：

- Gazebo、车体、控制器和基础真值里程计先稳定。
- `/mid360/raw_points`、`/livox/lidar`、`/livox/points`、`/livox/imu` 延后启动。
- `fast_lio2_map_frame_node.py` 使用固定 IMU pitch 和固定 map 水平补正，不再进行启动静止采样。
- `/localization/lio_odometry` 第一帧可用后，再启动 terrain mapper 和键盘直控。
- 默认不启动 Nav2 和 frontier exploration；只有显式设置 `ENABLE_NAV2=1` 才恢复自动导航链路。

### 干净终止旧进程

重启前先清理旧的 ROS/Gazebo 进程，尤其是上一次被中断或崩溃退出后：

```bash
cd /home/phragmund/LunarRover

for f in \
  log/manual_start/latest_gated.pid \
  log/manual_start/latest_manual_static.pid \
  log/manual_start/latest_clean_manual.pid
do
  if [ -f "$f" ]; then
    pid="$(cat "$f" 2>/dev/null || true)"
    [ -n "$pid" ] && kill -TERM -- "-$pid" 2>/dev/null || true
  fi
done

pkill -TERM -x rviz2 2>/dev/null || true
pkill -TERM -x ign 2>/dev/null || true
pkill -TERM -f "/usr/bin/ign gazebo" 2>/dev/null || true
pkill -TERM -f "/opt/ros/humble/bin/ros2 launch lunar_rover" 2>/dev/null || true
pkill -TERM -f "/opt/ros/humble/bin/ros2 run lunar_rover" 2>/dev/null || true
pkill -TERM -f "/home/phragmund/LunarRover/install/lunar_rover_.*/lib/lunar_rover_" 2>/dev/null || true
pkill -TERM -f "/home/phragmund/LunarRover/install/fast_lio/lib/fast_lio/fastlio_mapping" 2>/dev/null || true
pkill -TERM -f "/opt/ros/humble/lib/nav2_" 2>/dev/null || true
pkill -TERM -f "/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge" 2>/dev/null || true
sleep 3

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 daemon stop
```

确认没有残留：

```bash
pgrep -af "ign gazebo|rviz2|fastlio_mapping|fast_lio2_map_frame|terrain_mapper|chassis_cmd_mux|keyboard_rover_teleop|controller_server|bt_navigator|frontier_explorer"
```

正常情况下，除当前 `pgrep` 命令本身外，不应再看到旧进程。

### 一键启动当前推荐链路

推荐使用门控脚本启动。默认世界为 `reachable_maze`：

```bash
cd /home/phragmund/LunarRover
scripts/restart_gated_mapping_exploration_navigation.sh
```

`image_cave` 当前验证过的启动方式：

```bash
WORLD=image_cave SIM_SETTLE_SEC=20 scripts/restart_gated_mapping_exploration_navigation.sh
```

常用环境变量：

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `WORLD` | `reachable_maze` | Gazebo 世界名。`image_cave` 会自动使用 `spawn_z=1.45`。 |
| `SPAWN_X` / `SPAWN_Y` / `SPAWN_Z` / `SPAWN_YAW` | `-3.92` / `0.0` / 自动 / `0.0` | 车辆出生位姿。未设置 `SPAWN_Z` 时，`image_cave` 用 `1.45`，其他世界用 `0.45`。 |
| `MAP_ORIGIN_Z` | 同 `SPAWN_Z` | RViz world marker 的 map 原点高度。 |
| `SIM_SETTLE_SEC` | `12` | `/simulation/ground_truth/odometry` 可用后，启动传感器 bridge 和 Livox 驱动前的额外等待时间。 |
| `DELAY_LIVOX_DRIVERS` | `true` | 是否延迟启动 Gazebo 传感器 bridge 和 Mid-360 ROS 驱动。建议保持默认。 |
| `ENABLE_NAV2` | `false` | 是否启动 Nav2、命令 mux 和 frontier exploration。当前调试默认关闭。 |

如需临时恢复 Nav2 + frontier exploration：

```bash
ENABLE_NAV2=1 scripts/restart_gated_mapping_exploration_navigation.sh
```

启动顺序：

1. 启动 Gazebo、车体、控制器、`robot_state_publisher`、基础 bridge；此时 `livox_drivers:=false`、`sensor_bridges:=false`。
2. 等待 `/clock` 第一帧。
3. 等待 `/simulation/ground_truth/odometry` 第一帧，确认 Gazebo 车体真值里程计已经发布。
4. 额外等待 `SIM_SETTLE_SEC` 秒，让复杂地图和车体物理状态稳定。
5. 启动延迟的 Gazebo sensor bridge：`/mid360/raw_points`、`/mid360/raw_imu`、相机等话题开始进入 ROS。
6. 启动延迟的 Mid-360 ROS 驱动，发布 `/livox/lidar`、`/livox/points` 和 `/livox/imu`。
7. 等待 `/livox/lidar` 和 `/livox/imu` 第一帧。
8. 启动 `composite_wheel_imu_odometry_node.py`。
9. 启动 world model marker 和 RViz。
10. 启动 FAST-LIO2 定位链路。
11. `fast_lio2_map_frame_node.py` 使用固定 IMU pitch `35.273 deg` 替代启动 IMU 姿态采样。
12. `fast_lio2_map_frame_node.py` 使用固定 FAST-LIO map 水平补正 `roll=-0.58 deg, pitch=-0.67 deg, yaw=+0.007 deg`，不再采样启动姿态。
13. 脚本等待 `/localization/lio_odometry` 发布；只有这一步完成后，后续模块才启动。
14. 启动 terrain mapper。
15. 默认跳过 Nav2 和 frontier exploration。
16. 启动 direct keyboard teleop，直接发布到底盘 `/chassis/cmd_vel`。

仅当 `ENABLE_NAV2=1` 时，脚本才会启动 Nav2、等待 `/bt_navigator` active、启动经 mux 的键盘接管和 frontier exploration。

关键话题和控制关系：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/mid360/raw_points` | `sensor_msgs/msg/PointCloud2` | Gazebo sensor bridge 延迟启动后才出现的原始雷达点云 |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Mid-360 仿真驱动输出给 FAST-LIO2 的点云 |
| `/livox/points` | `sensor_msgs/msg/PointCloud2` | Mid-360 仿真驱动输出给 RViz/其他 ROS 节点的点云 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Mid-360 仿真 IMU |
| `/localization/lio_odometry_raw` | `nav_msgs/msg/Odometry` | FAST-LIO2 原始输出，保留算法内部估计姿态 |
| `/localization/lio_odometry` | `nav_msgs/msg/Odometry` | 经过固定水平补正后的导航定位输出 |
| `/localization/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 同步应用固定 map 水平补正后的注册点云 |
| `/localization/lio_map_cloud` | `sensor_msgs/msg/PointCloud2` | 同步应用固定 map 水平补正后的 FAST-LIO2 地图点云 |
| `/chassis/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 默认键盘直控输出，真正发送给底盘控制器的指令 |
| `/navigation/chassis_cmd_vel` | `geometry_msgs/msg/TwistStamped` | 仅 `ENABLE_NAV2=1` 时存在，Nav2 经 `twist_stamper` 后的底盘指令 |
| `/keyboard/chassis_cmd_vel` | `geometry_msgs/msg/TwistStamped` | 仅 `ENABLE_NAV2=1` 时用于 mux 的键盘控制指令 |

默认键盘控制逻辑：

- Nav2 不启动，`chassis_cmd_mux_node.py` 不启动。
- direct keyboard teleop 直接发布 `/chassis/cmd_vel`。
- `ENABLE_NAV2=1` 时才恢复 mux 接管逻辑：键盘 5 秒内有输入则覆盖导航，5 秒无输入后恢复导航指令。

启动完成后，日志中应出现类似顺序：

```text
starting sim: world=image_cave delayed_livox_drivers=true
Gazebo ground truth is live; waiting 20s before Livox ROS drivers
starting delayed Gazebo sensor bridge
starting delayed Mid-360 ROS point cloud driver
starting delayed Mid-360 IMU driver
starting FAST-LIO localization with fixed map leveling
Using fixed FAST-LIO level correction
FAST-LIO corrected odometry is live; starting terrain mapping
skipping Nav2/frontier exploration; starting direct keyboard teleop on /chassis/cmd_vel
```

如果要终止当前链路，优先按脚本所在进程组整体停止：

```bash
cd /home/phragmund/LunarRover
pid="$(cat log/manual_start/latest_gated.pid 2>/dev/null || pgrep -f restart_gated_mapping_exploration_navigation.sh | head -n 1)"
[ -n "$pid" ] && kill -TERM -- "-$pid"
```

也可以先查进程：

```bash
pgrep -af "restart_gated|fastlio_mapping|rviz2|ign gazebo|terrain_mapper|keyboard_rover_teleop"
```

注意：

- 默认链路不再等待启动静止采样；FAST-LIO 后端自身 `IMU Initial Done` 出现前仍不要移动机器人。
- `DELAY_LIVOX_DRIVERS=true` 只延迟 ROS 侧 sensor bridge 和 Mid-360 ROS 驱动。Gazebo URDF 中 `<always_on>true</always_on>` 的传感器本体仍可能在 Gazebo 内部运行。
- 当前默认不启动 Nav2，因为 costmap/terrain layer 仍在调试中。确认 costmap 可用后再用 `ENABLE_NAV2=1` 恢复自动导航链路。
- `src/lunar_rover_bringup/launch/fast_lio2_exploration_navigation.launch.py` 仍可用于调试，但当前推荐入口是 `scripts/restart_gated_mapping_exploration_navigation.sh`。
- direct keyboard teleop 默认自动发现 `/dev/input/by-id/*event-kbd`、`/dev/input/by-path/*event-kbd` 和系统 keyboard event 设备。只有需要限定某一把键盘时，才手动给 `scripts/run_direct_keyboard_teleop.sh --device ...`。

## 启动基础仿真

```bash
ros2 launch lunar_rover_bringup bringup.launch.py \
  mode:=sim gui:=true rviz:=true sensor_noise:=false
```

可用世界：

```text
flat
steps
rocks
mixed
stairs_maze
reachable_maze
lava_tube
image_cave
terrain_preview
```

默认测试世界是 `image_cave`。该地图包含从参考洞穴图生成的封闭墙体拓扑和
0.2 m 边长、0.2 m 高差范围的三角伪曲面地形。地图内容见
[`docs/simulation_worlds.md`](docs/simulation_worlds.md)。

只启动Gazebo模型和原始桥接：

```bash
ros2 launch lunar_rover_simulation simulation.launch.py \
  gui:=true rviz:=true sensor_noise:=false
```

实车原始传感器入口：

```bash
ros2 launch lunar_rover_bringup bringup.launch.py mode:=real
```

## 原始数据查看

查看雷达点云：

```bash
rviz2 -d \
  /home/phragmund/LunarRover/install/lunar_rover_description/share/\
lunar_rover_description/rviz/lunar_rover.rviz
```

查看IMU消息：

```bash
ros2 topic echo /livox/imu
```

查看彩色图和原始深度图：

```bash
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
ros2 run rqt_image_view rqt_image_view /camera/depth/image_raw
```

## 工程边界

后续SLAM研究应新建独立ROS包，并只订阅本基础环境提供的原始传感器话题。新算法不应修改车辆模型、Gazebo桥或基础控制包，除非传感器外参或底盘接口本身经过独立验证确实有误。
