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

## 成熟启动链路：FAST-LIO2建图 + 探索 + 导航 + 键盘接管

推荐使用门控脚本启动完整链路：

```bash
cd /home/phragmund/LunarRover
scripts/restart_gated_mapping_exploration_navigation.sh
```

这个脚本是当前成熟启动流程。它不会按固定秒数直接启动导航，而是先等待 FAST-LIO2 输出已经过启动水平校正的 `/localization/lio_odometry` 第一帧，再启动地形建图、Nav2、键盘控制和探索。这样可以避免车辆在 FAST-LIO2 启动校平前被自动导航或键盘移动，导致 `map`、`odom`、`base_link` 的 TF 被错误初始姿态污染。

启动顺序：

1. 启动 Gazebo、原始传感器、底盘控制、`robot_state_publisher`。
2. 启动 `composite_wheel_imu_odometry_node.py`，提供轮速 + IMU 组合里程计。
3. 启动透明度 30% 的仿真 world model marker 和 RViz。
4. 静置约 6 秒后启动 FAST-LIO2 定位链路。
5. `fast_lio2_map_frame_node.py` 从 `/livox/imu` 读取启动 IMU 姿态，使用真实启动 pitch 替代固定安装角；随后等待 FAST-LIO2 raw odom 初步收敛。
6. `fast_lio2_map_frame_node.py` 采样启动时未校正的 `map->base_link` 姿态，自动生成固定水平校正，只清除启动 roll/pitch，保留 yaw。
7. 脚本等待 `/localization/lio_odometry` 发布。只有这一步完成后，才允许后续模块启动。
8. 启动 terrain mapper。
9. 启动 Nav2 和 `chassis_cmd_mux`。
10. 等 `/bt_navigator` lifecycle 变为 active。
11. 启动 direct keyboard teleop。
12. 启动 frontier exploration。

关键话题和控制关系：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/localization/lio_odometry_raw` | `nav_msgs/msg/Odometry` | FAST-LIO2 原始输出，保留算法内部估计姿态 |
| `/localization/lio_odometry` | `nav_msgs/msg/Odometry` | 经过启动水平校正后的导航定位输出 |
| `/localization/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 同步应用 map 水平校正后的注册点云 |
| `/localization/lio_map_cloud` | `sensor_msgs/msg/PointCloud2` | 同步应用 map 水平校正后的 FAST-LIO2 地图点云 |
| `/navigation/chassis_cmd_vel` | `geometry_msgs/msg/TwistStamped` | Nav2 经 `twist_stamper` 后的底盘指令 |
| `/keyboard/chassis_cmd_vel` | `geometry_msgs/msg/TwistStamped` | 键盘控制指令 |
| `/chassis/cmd_vel` | `geometry_msgs/msg/TwistStamped` | mux 后真正发送给底盘控制器的指令 |

键盘接管逻辑：

- `chassis_cmd_mux_node.py` 订阅 `/navigation/chassis_cmd_vel` 和 `/keyboard/chassis_cmd_vel`。
- 键盘 5 秒内有输入时，mux 输出键盘指令，导航仍运行但失去车体控制权。
- 键盘 5 秒没有输入后，mux 自动恢复导航指令输出。

启动完成后，日志中应出现类似顺序：

```text
Startup FAST-LIO level correction calibrated
FAST-LIO corrected odometry is live; starting terrain mapping
starting Nav2 and command mux after level correction
Nav2 lifecycle bringup complete
starting direct keyboard teleop
starting frontier exploration
```

如果要终止当前链路，优先按脚本所在进程组整体停止；也可以先查进程：

```bash
pgrep -af "restart_gated|fastlio_mapping|rviz2|ign gazebo|controller_server|frontier|keyboard_rover_teleop"
```

注意：

- 启动后在日志出现 `FAST-LIO corrected odometry is live` 之前不要移动机器人。
- 不建议用固定 TimerAction 直接启动导航，因为 Nav2 可能在 FAST-LIO2 校平前控制车体运动。
- `src/lunar_rover_bringup/launch/fast_lio2_exploration_navigation.launch.py` 仍可用于调试，但当前推荐入口是 `scripts/restart_gated_mapping_exploration_navigation.sh`。
- direct keyboard teleop 默认读取 `/dev/input/event3,/dev/input/event6,/dev/input/event9,/dev/input/event13`。如本机键盘设备变化，需要同步修改脚本中的 `--device` 参数。

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
