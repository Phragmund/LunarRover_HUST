# ROS 2 包结构

`src/` 是 LunarRover 的 ROS 2 工作区源码目录。仓库按“接口、模型、仿真、传感器、控制、定位、建图、导航、探索、集成启动”拆包，目的是让基础底盘能力和上层算法可以独立替换。

## 基础分层

```text
lunar_rover_msgs
  -> lunar_rover_control
  -> lunar_rover_simulation

lunar_rover_description
  -> lunar_rover_simulation
  -> lunar_rover_bringup

lunar_rover_sensors
  -> lunar_rover_bringup
```

完整探索运行链路：

```text
lunar_rover_bringup
  -> lunar_rover_mapping/composite_wheel_imu_odometry_node.py
  -> lunar_rover_localization/fast_lio2_localization.launch.py
  -> lunar_rover_mapping/terrain_mapping.launch.py
  -> lunar_rover_navigation/navigation.launch.py
  -> lunar_rover_exploration/frontier_exploration.launch.py
```

实际 launch 中会有交叉组合，例如 `lunar_rover_bringup` 启动仿真、传感器和控制，完整探索脚本再按就绪状态启动 localization、mapping、navigation 和 exploration。

## 包目录

| 目录 | 说明 |
| --- | --- |
| `lunar_rover_msgs/` | 公共消息定义，目前提供 `ChassisState.msg`。 |
| `lunar_rover_description/` | 六轮摇臂月球车 URDF/Xacro、模型 RViz 配置和独立 display launch。 |
| `lunar_rover_simulation/` | Gazebo Fortress 世界、模型、bridge 配置、仿真 Mid-360 转换节点和世界 marker。 |
| `lunar_rover_sensors/` | 仿真/实车传感器启动入口，保持原始话题语义。 |
| `lunar_rover_control/` | ros2_control 底盘控制器、摇臂差动控制器、仿真底盘状态和轮反馈里程计。 |
| `lunar_rover_mapping/` | 组合轮速 IMU 里程计、FAST-LIO2 点云地形建图、旧 SLAM/OctoMap 调试入口。 |
| `lunar_rover_localization/` | FAST-LIO2 输入/输出适配、固定 map 水平补正和 `map->odom` TF 桥。 |
| `lunar_rover_navigation/` | Nav2 配置、地形 costmap layer、`Twist` 到底盘 `TwistStamped` 适配、键盘接管 mux。 |
| `lunar_rover_exploration/` | 地形感知前沿探索节点，向 Nav2 发送 `NavigateToPose` 目标。 |
| `lunar_rover_bringup/` | 基础 bringup 和完整 FAST-LIO2 探索导航调试 launch。 |

## 约定

- 下层包不依赖上层算法包，例如 `description`、`simulation`、`sensors` 和 `control` 不应依赖 `navigation` 或 `exploration`。
- `/simulation/ground_truth/*` 只能用于验证、标定和调试，不应进入算法闭环。
- 新增目标识别、机械臂、采样状态机时，优先新增独立包，再通过 launch 或任务调度包集成。
