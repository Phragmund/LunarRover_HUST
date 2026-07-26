# 六轮摇臂月球车 ROS 2/Gazebo 导航感知框架

本仓库是一套面向六轮摇臂月球车的 ROS 2 Humble 框架，覆盖 Gazebo Fortress 可视化仿真、Mid-360 激光雷达、Gemini 335 深度相机、地形感知、FAST-LIO/RTAB-Map 建图、Nav2 导航、安全限速和任务记录。仿真世界使用标准地球重力 `9.80665 m/s^2`，地形包括平地、台阶、岩石障碍和混合路线。

## 系统边界

上位机只发布底盘运动控制指令，不直接驱动单个电机。实车下位机负责 4WS6WD 电机分配、闭环驱动、急停、故障处理，并向上位机回传完整关节状态和底盘状态。接口契约见 [lower_controller_contract.md](/home/phragmund/LunarRover/docs/lower_controller_contract.md)。

仿真模式下，仓库内的 `ros2_control` 控制器代替下位机，用于验证运动学、导航链路和感知算法。实车模式下不会启动仿真控制器，只启动 `robot_state_publisher`、真实传感器、轮速里程计、定位建图、感知导航和安全监督节点。

## 主要话题

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 上位机到下位机 | `/chassis/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 安全监督后的底盘速度指令 |
| 下位机到上位机 | `/joint_states` | `sensor_msgs/msg/JointState` | 6 个车轮、4 个转向、摇臂/摆臂关节状态 |
| 下位机到上位机 | `/chassis/state` | `lunar_rover_msgs/msg/ChassisState` | 模式、故障、急停和看门狗状态 |
| 激光点云 | `/livox/points` | `sensor_msgs/msg/PointCloud2` | 仿真适配或 Livox 驱动转换输出 |
| 深度点云 | `/camera/depth_registered/points` | `sensor_msgs/msg/PointCloud2` | Gemini 335 注册深度点云 |
| 地形代价 | `/terrain/traversability` | `nav_msgs/msg/OccupancyGrid` | `0..99` 可通行代价，`100` 不可通行 |
| 定位输出 | `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF 融合里程计和 LIO |
| Nav2 安全输出 | `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | Nav2 collision monitor 输出 |

## 依赖和构建

```bash
./scripts/bootstrap_humble.sh
source /opt/ros/humble/setup.bash
./scripts/import_dependencies.sh
./scripts/import_hardware_dependencies.sh
./scripts/build.sh
source install/setup.bash
```

`dependencies/core.repos` 引入 FAST-LIO 和 Livox ROS 驱动，`dependencies/sdk.repos` 引入 Livox SDK2，`dependencies/hardware.repos` 引入 Orbbec SDK ROS 2 驱动。FAST-LIO 的 Humble 适配补丁在 [fast_lio_humble.patch](/home/phragmund/LunarRover/patches/fast_lio_humble.patch)。

## 仿真运行

```bash
ros2 launch lunar_rover_bringup bringup.launch.py mode:=sim world:=mixed gui:=true rviz:=true
```

可选世界：

```bash
world:=flat
world:=steps
world:=rocks
world:=mixed
```

带航点任务和指标记录：

```bash
MISSION="$(ros2 pkg prefix --share lunar_rover_missions)/config/mixed_course.yaml"
ros2 launch lunar_rover_bringup bringup.launch.py \
  mode:=sim world:=mixed mission_file:="$MISSION" \
  record_metrics:=true metrics_file:=~/.ros/lunar_rover_metrics.json
```

## 实车运行

```bash
ros2 launch lunar_rover_bringup bringup.launch.py mode:=real rviz:=true
```

实车运行前需要确认：

- Livox Mid-360 网络、`src/lunar_rover_sensors/config/livox_mid360.json` 和主机 IP 匹配。
- Orbbec Gemini 335 驱动和 udev 规则已安装，驱动包名为 `orbbec_camera`。
- 下位机已经通过 ROS 2 或 micro-ROS 暴露 `/joint_states`、`/chassis/state`，并订阅 `/chassis/cmd_vel`。
- 所有传感器和下位机使用同一时间源；跨主机运行时需要 NTP/chrony/PTP。

## 算法链路

1. Gazebo 或真实传感器发布 Mid-360 点云/IMU 和 Gemini 335 深度数据。
2. FAST-LIO 输出 LIO 里程计，轮速里程计来自 `/joint_states`。
3. EKF 融合轮速与 LIO，`pose_tf_splitter` 发布 `odom -> base_footprint -> base_link`。
4. RTAB-Map 在线建图，负责 `map -> odom`。
5. 地形感知节点融合激光和深度点云，生成局部 2.5D 高程、坡度、粗糙度、台阶高度和 traversability。
6. Nav2 代价地图加载 `lunar_rover_navigation::TerrainLayer`，台阶和岩石会进入规划代价。
7. Nav2 velocity smoother、collision monitor 和安全监督节点依次过滤速度指令，最终只向下位机发布 `/chassis/cmd_vel`。

## 当前验证

本仓库包含不依赖 ROS 安装的静态验证：

```bash
python3 tools/validate_workspace.py
python3 -m py_compile $(rg --files -g '*.py')
bash -n scripts/bootstrap_humble.sh scripts/import_dependencies.sh scripts/import_hardware_dependencies.sh scripts/build.sh
```

在安装 ROS 2 后运行完整构建和测试：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## 仿真限制

Mid-360 和 Gemini 335 的仿真是算法级近似，不是厂商固件级仿真。Gazebo 使用 GPU lidar/RGBD camera 加噪声和丢点模型来验证定位、建图、地形感知和导航闭环；实机性能仍需要在真实传感器、真实下位机和真实地形上重新标定阈值。
