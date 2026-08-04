# 六轮摇臂月球车 ROS 2/Gazebo 基础研究环境

本仓库当前是一个不包含任何SLAM或自主导航算法的干净基础环境，用于重新开展定位、建图和导航研究。

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

## 明确不包含

当前主分支不包含：

- 点云滤波和IMU滤波；
- 轮速里程计；
- ICP、LIO和视觉里程计；
- EKF或其他传感器融合；
- 二维或三维建图；
- 回环检测和地图优化；
- 地形可通行性分析；
- Nav2、路径规划和避障；
- 循墙、上台阶、往返测试等自动任务。

一阶段算法实验已保存在Git标签：

```text
backup-before-clean-slam-2026-07-27
```

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
