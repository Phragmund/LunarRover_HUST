# lunar_rover_bringup

集成启动包，负责把模型、仿真/实车传感器、底盘控制和上层算法 launch 组合起来。

## Launch

| 文件 | 作用 |
| --- | --- |
| `launch/bringup.launch.py` | 基础入口。`mode:=sim` 启动 Gazebo、模型、控制器和可选仿真传感器；`mode:=real` 启动 `robot_state_publisher` 和实车传感器。 |
| `launch/fast_lio2_exploration_navigation.launch.py` | FAST-LIO2、地形建图、Nav2 和前沿探索的调试 launch。当前推荐日常完整链路仍是根目录脚本 `scripts/restart_gated_mapping_exploration_navigation.sh`。 |

## 参数边界

`bringup.launch.py` 只负责选择仿真/实车模式和传递世界、GUI、传感器噪声、出生位姿等基础参数。定位、建图、导航和探索的具体参数放在对应 ROS 包的 `config/` 下。

仿真模式下，`bringup.launch.py` 会向 `lunar_rover_simulation/launch/simulation.launch.py` 透传两个传感器门控参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `livox_drivers` | `true` | 是否启动 Mid-360 ROS 点云/IMU 驱动，输出 `/livox/lidar`、`/livox/points`、`/livox/imu`。 |
| `sensor_bridges` | `true` | 是否桥接 Gazebo 传感器话题到 ROS，包括 `/mid360/raw_points`、`/mid360/raw_imu` 和相机话题。 |

根目录脚本 `scripts/restart_gated_mapping_exploration_navigation.sh` 在默认延迟模式下会先以 `livox_drivers:=false sensor_bridges:=false` 启动基础仿真，等待 Gazebo ground truth 稳定后再单独启动传感器 bridge 和 Mid-360 驱动。
