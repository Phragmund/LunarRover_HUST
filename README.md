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
  -> 可选 Mid-360/Gemini互补校正 -> /localization/cloud_fused
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
- [`docs/real_chassis_test.md`](docs/real_chassis_test.md)：实车底盘预检、安全使能和启动步骤。
- [`docs/real_arm_visual_grasping_handoff.md`](docs/real_arm_visual_grasping_handoff.md)：实车机械臂视觉抓取、手眼标定、目标三维定位和 RViz 观察交接。
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
| `/localization/cloud_fused` | `sensor_msgs/msg/PointCloud2` | 雷达约束、相机补充后的融合点云 |
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

非交互 agent 后台启动时建议使用独立进程组和日志文件：

```bash
setsid -f bash -lc 'cd /home/phragmund/LunarRover && source /opt/ros/humble/setup.bash && source install/setup.bash && exec env WORLD=image_cave SIM_SETTLE_SEC=20 scripts/restart_gated_mapping_exploration_navigation.sh' \
  </dev/null >/tmp/lunar_rover_gated_restart.log 2>&1
```

常用环境变量：

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `WORLD` | `reachable_maze` | Gazebo 世界名。`image_cave` 会自动使用 `spawn_z=1.45`。 |
| `SPAWN_X` / `SPAWN_Y` / `SPAWN_Z` / `SPAWN_YAW` | `-3.92` / `0.0` / 自动 / `0.0` | 车辆出生位姿。未设置 `SPAWN_Z` 时，`image_cave` 用 `1.45`，其他世界用 `0.45`。 |
| `MAP_ORIGIN_Z` | 同 `SPAWN_Z` | RViz world marker 的 map 原点高度。`TRUTH_LOCALIZATION_MODE=replace_fastlio` 且未手动设置时，脚本会在车辆落稳后用 `/simulation/ground_truth/odometry.pose.pose.position.z - TRUTH_BASE_HEIGHT_ABOVE_GROUND` 估计地面高度，避免把 spawn 初始高度或 base_link 高度当成地图零点。 |
| `TRUTH_BASE_HEIGHT_ABOVE_GROUND` | `0.1825` | 真值替换模式下 base_link 到地面的标称高度，来自 URDF `wheel_center_z=-0.12` 和 `wheel_radius=0.0625`。 |
| `SIM_SETTLE_SEC` | `12` | `/simulation/ground_truth/odometry` 可用后，启动传感器 bridge 和 Livox 驱动前的额外等待时间。 |
| `CLOCK_TIMEOUT_SEC` | `image_cave` 为 `180`，其他世界为 `90` | 等待 `/clock` 第一帧的最长时间。复杂 Gazebo 世界加载和 controller_manager 激活可能超过 60 秒。 |
| `GROUND_TRUTH_TIMEOUT_SEC` | `image_cave` 为 `180`，其他世界为 `90` | 等待 `/simulation/ground_truth/odometry` 第一帧的最长时间。 |
| `CONTROLLER_TIMEOUT_SEC` | `image_cave` 为 `180`，其他世界为 `90` | 等待 `joint_state_broadcaster`、`rocker_differential_controller` 和 `rover_controller` 全部 active 的最长时间。 |
| `DELAY_LIVOX_DRIVERS` | `true` | 是否延迟启动 Gazebo 传感器 bridge 和 Mid-360 ROS 驱动。建议保持默认。 |
| `ENABLE_CLOUD_FUSION` | `false` | 是否启用 Gemini 深度点云与 Mid-360 点云互补融合。默认关闭，terrain mapper 和 elevation grid 直接使用 `/localization/cloud_registered`；设为 `1` 时启动融合节点并改用 `/localization/cloud_fused`。 |
| `ENABLE_COSTMAP` | 非真值模式 `true`，真值模式 `false` | 是否启动 Nav2 costmap stack 和命令 mux，让 RViz `/global_costmap/costmap` 有发布者。非真值默认开启，但不会自动发送导航目标。 |
| `ENABLE_NAV2` | `false` | 是否启动 frontier exploration 或 cruise 自动目标。`ENABLE_NAV2=1` 会隐含启动 costmap stack。 |
| `TRUTH_LOCALIZATION_MODE` | `off` | 真值调试模式。`replace_fastlio` 用 Gazebo 真值完全替换 FAST-LIO。 |
| `USE_TRUTH_LOCALIZATION` | `false` | 兼容旧开关。设为 `1` 等价于 `TRUTH_LOCALIZATION_MODE=replace_fastlio`。 |

如需临时恢复 frontier exploration 自动目标：

```bash
ENABLE_NAV2=1 scripts/restart_gated_mapping_exploration_navigation.sh
```

启动顺序：

1. 启动 Gazebo、车体、控制器、`robot_state_publisher`、基础 bridge；此时 `livox_drivers:=false`、`sensor_bridges:=false`。
2. 等待 `/clock` 第一帧。
3. 等待 `/simulation/ground_truth/odometry` 第一帧，确认 Gazebo 车体真值里程计已经发布。
4. 等待 `joint_state_broadcaster`、`rocker_differential_controller` 和 `rover_controller` 全部 active。
5. 额外等待 `SIM_SETTLE_SEC` 秒，让复杂地图和车体物理状态稳定。
6. 启动延迟的 Gazebo sensor bridge：`/mid360/raw_points`、`/mid360/raw_imu`、相机等话题开始进入 ROS。
7. 启动延迟的 Mid-360 ROS 驱动，发布 `/livox/lidar`、`/livox/points` 和 `/livox/imu`。
8. 等待 `/livox/lidar` 和 `/livox/imu` 第一帧。
9. 启动 `composite_wheel_imu_odometry_node.py`。
10. 启动 world model marker 和 RViz。
11. 启动 FAST-LIO2 定位链路。
12. `fast_lio2_map_frame_node.py` 使用固定 IMU pitch `35.273 deg` 替代启动 IMU 姿态采样。
13. `fast_lio2_map_frame_node.py` 使用固定 FAST-LIO map 水平补正 `roll=-0.58 deg, pitch=-0.67 deg, yaw=+0.007 deg`，不再采样启动姿态。
14. 脚本等待 `/localization/lio_odometry` 发布；只有这一步完成后，后续模块才启动。
15. 启动 terrain mapper。
16. 非真值模式默认启动 Nav2 costmap stack 和 command mux，使 RViz `/global_costmap/costmap` 可见。
17. 默认不启动 frontier exploration 或 cruise 自动目标。
18. 启动 direct keyboard teleop；costmap stack 运行时发布到 `/keyboard/chassis_cmd_vel` 并经 mux 输出到底盘。

`ENABLE_COSTMAP=0` 时脚本会完全跳过 Nav2 costmap stack，键盘控制直接发布到底盘 `/chassis/cmd_vel`。仅当 `ENABLE_NAV2=1` 时，脚本才会在 costmap stack active 后启动 frontier exploration；同时设置 `ENABLE_CRUISE=1` 时改为启动 image cave waypoint cruise。

关键话题和控制关系：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/mid360/raw_points` | `sensor_msgs/msg/PointCloud2` | Gazebo sensor bridge 延迟启动后才出现的原始雷达点云 |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Mid-360 仿真驱动输出给 FAST-LIO2 的点云 |
| `/livox/points` | `sensor_msgs/msg/PointCloud2` | Mid-360 仿真驱动输出给 RViz/其他 ROS 节点的点云 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Mid-360 仿真 IMU |
| `/localization/lio_odometry_raw` | `nav_msgs/msg/Odometry` | FAST-LIO2 原始输出，保留算法内部估计姿态 |
| `/localization/lio_odometry` | `nav_msgs/msg/Odometry` | 经过固定水平补正后的导航定位输出 |
| `/localization/composite_odometry` | `nav_msgs/msg/Odometry` | 轮速+IMU组合里程计输出，供 `map_odom_tf_bridge` 使用 |
| `/localization/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 同步应用固定 map 水平补正后的注册点云 |
| `/localization/cloud_fused` | `sensor_msgs/msg/PointCloud2` | 仅 `ENABLE_CLOUD_FUSION=1` 时发布。Mid-360约束Gemini深度偏差、剔除冲突并补足雷达空隙后的点云 |
| `/localization/lio_map_cloud` | `sensor_msgs/msg/PointCloud2` | 同步应用固定 map 水平补正后的 FAST-LIO2 地图点云 |
| `/chassis/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 真正发送给底盘控制器的指令。costmap stack 启动时由 mux 输出；`ENABLE_COSTMAP=0` 时由键盘节点直接输出。 |
| `/navigation/chassis_cmd_vel` | `geometry_msgs/msg/TwistStamped` | Nav2 经 `twist_stamper` 后的底盘指令，costmap stack 启动时存在。 |
| `/keyboard/chassis_cmd_vel` | `geometry_msgs/msg/TwistStamped` | costmap stack 启动时用于 mux 的键盘控制指令。 |

默认键盘控制逻辑：

- 非真值默认启动 Nav2 costmap stack 和 `chassis_cmd_mux_node.py`，direct keyboard teleop 发布 `/keyboard/chassis_cmd_vel`。
- `ENABLE_COSTMAP=0` 时不启动 mux，direct keyboard teleop 直接发布 `/chassis/cmd_vel`。
- 键盘 5 秒内有输入则覆盖导航输出，5 秒无输入后恢复导航指令；默认没有 frontier/cruise 目标，因此不会自动行驶。

启动完成后，日志中应出现类似顺序：

```text
starting sim: world=image_cave delayed_livox_drivers=true
received first /clock message
received first /simulation/ground_truth/odometry message
rover controllers are active
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
- `image_cave` 下 Gazebo 和 `controller_manager` 有时会在前 60 秒内响应很慢，脚本已经把 `/clock`、ground truth 和控制器等待改成可恢复短轮询，并把 `image_cave` 默认超时提高到 180 秒。看到 spawner 早期的 `switch_controller` 超时不一定表示启动失败，只要后续日志出现 `rover controllers are active` 就可以继续。
- `DELAY_LIVOX_DRIVERS=true` 只延迟 ROS 侧 sensor bridge 和 Mid-360 ROS 驱动。Gazebo URDF 中 `<always_on>true</always_on>` 的传感器本体仍可能在 Gazebo 内部运行。
- 当前默认不启动 Nav2，因为 costmap/terrain layer 仍在调试中。确认 costmap 可用后再用 `ENABLE_NAV2=1` 恢复自动导航链路。
- `src/lunar_rover_bringup/launch/fast_lio2_exploration_navigation.launch.py` 仍可用于调试，但当前推荐入口是 `scripts/restart_gated_mapping_exploration_navigation.sh`。
- direct keyboard teleop 默认自动发现 `/dev/input/by-id/*event-kbd`、`/dev/input/by-path/*event-kbd` 和系统 keyboard event 设备。只有需要限定某一把键盘时，才手动给 `scripts/run_direct_keyboard_teleop.sh --device ...`。
- 如果启动后拔插过键盘，旧的 `keyboard_rover_teleop` 可能还在读取已经失效的 `/dev/input/event* (deleted)` 句柄，表现为节点存在但按键没有任何 `/chassis/cmd_vel` 输出。此时不要重启 Gazebo/FAST-LIO/建图，只需重启键盘节点：`pkill -f keyboard_rover_teleop.py || true`，然后在当前推荐链路中运行 `scripts/run_direct_keyboard_teleop.sh --topic /chassis/cmd_vel`。详细排查见 [`docs/keyboard_teleop_direct.md`](docs/keyboard_teleop_direct.md)。
- 真值替换模式下如果快速转弯出现扇形发散，优先检查 LiDAR stamp 与 Gazebo truth pose 是否按时间对齐，以及 terrain mapper 是否持续 TF extrapolation。排查和修复记录见 [`docs/truth_localization_sync_debugging.md`](docs/truth_localization_sync_debugging.md)。

## 实车验证启动链路：Livox + FAST-LIO2 + H7 + 手机遥控

记录时间：2026-08-23 23:49 CST。该链路已在实车接线后验证可启动，当前用于实车
雷达建图和手机遥控底盘测试。

当前实车手动控制入口是：

```text
web_joystick_server.py
  -> /manual_control (lunar_rover_msgs/ManualControl)
  -> motion_solver.py
  -> H7 USB CDC (/dev/ttyACM0)
  -> 3508 + 舵轮
```

不要在这条实车手动链路里启动旧的
`chassis_driver_node.py + chassis_cmd_mux_node.py + /chassis/cmd_vel`。旧链路保留给
仿真/Nav 调试和后续自动导航兼容，不作为当前上车手动控制入口。

### 启动前检查

1. H7 USB CDC 已连接，通常为 `/dev/ttyACM0`。
2. Livox Mid-360 以太网已连接，上位机有 `192.168.2.50/24`，雷达地址为
   `192.168.2.204`。
3. 手机和上位机在同一热点/Wi-Fi。当前验证时手机热点网段为
   `192.168.43.0/24`，上位机 Wi-Fi 地址为 `192.168.43.113`。
4. 车辆静置，FAST-LIO 和 map 水平校正完成前不要移动车体。
5. `motion_solver.py` 启动即向 H7 发送 `$M3508:ENABLE,1`。启动前确认车轮悬空或
   周围清空；手机摇杆一推，3508 就会响应。

确认没有残留进程：

```bash
pgrep -af "ros2|rviz2|motion_solver.py|keyboard_controller.py|web_joystick_server.py|chassis_driver_node.py|chassis_cmd_mux_node.py|livox|fastlio|robot_state_publisher"
```

### 一键启动

推荐固定一个短 token，避免手机输入长随机 token：

```bash
cd /home/phragmund/LunarRover
REAL_CHASSIS_H7_PORT=/dev/ttyACM0 \
WEB_JOYSTICK_TOKEN=r1234 \
RUN_REAL_CHASSIS=true \
RUN_RVIZ=true \
scripts/run_real_lidar_mapping.sh
```

启动后终端应打印手机地址，例如：

```text
Phone controller: http://192.168.43.113:8000/?token=r1234
```

手机必须打开完整 URL，包括 `?token=r1234`。如果日志出现 `WebSocket /ws 403`，
说明手机页面没有带 token、token 为空，或旧标签页还在重连。关掉旧标签，只保留
完整 URL 页面即可。

### 成功标志

启动日志中应依次出现：

```text
motion_solver: 已使能 3508，等待控制指令...
web_joystick_server.py: Web joystick server 已启动，发布话题: /manual_control
livox_ros_driver2_node: successfully enable Livox Lidar imu
fastlio_mapping: IMU Initial Done
fastlio_mapping: Initialize the map kdtree
fast_lio2_map_frame: Calibrated effective IMU mount attitude ...
fast_lio2_map_frame: Startup FAST-LIO level correction calibrated ...
```

2026-08-23 实测校平结果示例：

```text
Calibrated effective IMU mount attitude: roll=+0.463 deg, pitch=+54.273 deg
Startup FAST-LIO level correction calibrated:
  measured rpy=(-0.221, +0.309, -0.166) deg
  correction rpy=(+0.221, -0.310, -0.001) deg
```

验证话题：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /localization/lio_odometry --once
ros2 topic echo /livox/imu --once
ros2 topic echo /manual_control --once
```

`/manual_control` 在手机页面连接后应持续发布；未操作摇杆时杆量为 0。

### 实车自主抓取（2026-08-24）

完整实车启动现在默认同时启动 Gemini、MoveIt、持续目标检测、RGB-D 支撑面定位、
机械臂轨迹接口和键盘监听。首次使用前安装 MoveIt 2：

```bash
sudo apt-get update
sudo apt-get install ros-humble-moveit
```

启动命令保持不变。FAST-LIO 校平完成、机械臂周围清空并确认目标在工作空间内后，按
一次 `C` 进入抓取模式。状态变化可通过以下话题检查：

```bash
ros2 topic echo /grasp/status
ros2 topic echo /grasp/active
ros2 topic echo /perception/target_observation
ros2 topic echo /search/target_confirmed
ros2 topic echo /joint_states
```

抓取期间系统会取消当前 Nav2 目标，`motion_solver` 和 `chassis_cmd_mux` 同时锁止底盘，
并停止向 FAST-LIO 转发 `/livox/lidar_raw` 点云；IMU仍按原时间戳连续转发，但抓取期间
固定为进入抓取前的重力值和零角速度，避免恢复时出现时间断层。已建立的地图和进入抓取
时的定位结果因此近似冻结。抓取已分为预抓取、粗视觉校正、近端精校正、闭合和撤离，
支持按支撑面法向区分地面目标与机械臂可达方向内的墙面目标；当前仍不执行存放。

未配置自定义权重时，视觉链路使用高饱和度实例作为联调后备；真实岩石样本必须使用
自定义实例分割权重。主动摆臂和自动触发抓取默认关闭，常规行驶期间的视觉检测与多帧
确认保持运行。模型训练、墙面判别、分阶段伺服和 FoundationPose 接口详见
[target_perception_and_foundationpose.md](docs/target_perception_and_foundationpose.md)。参数位于
`src/lunar_rover_manipulation/config/grasping.yaml`。Gemini 末端安装初值为
`xyz="0.04 0 0.065" rpy="0 -1.57079632679 0"`，正式抓取前必须按实车安装结果完成
手眼外参测量；可通过环境变量 `ARM_CAMERA_XYZ`、`ARM_CAMERA_RPY` 覆盖。暂时不测试
抓取时可设置：

```bash
RUN_GRASPING=false RUN_KEYBOARD=false scripts/run_real_lidar_mapping.sh
```

### 只重启手机遥控

如果完整链路已经在跑，只是 token 输入不方便，不要重启雷达和 FAST-LIO。只停
`web_joystick_server.py`，再用短 token 单独启动：

```bash
pkill -INT -f web_joystick_server.py
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lunar_rover_navigation web_joystick_server.py --ros-args \
  -p port:=8000 \
  -p access_token:=r1234
```

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
