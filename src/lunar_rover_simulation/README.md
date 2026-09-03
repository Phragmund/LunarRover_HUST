# lunar_rover_simulation

Gazebo Fortress 仿真包，提供世界、模型、ROS-Gazebo bridge、控制器配置和 Mid-360 适配节点。

```bash
ros2 launch lunar_rover_simulation simulation.launch.py world:=reachable_maze
```

`sensor_bridges` 与 `livox_drivers` 可分别控制高频传感器 bridge 和 Livox ROS 适配器，以支持分阶段启动。所有世界使用标准地球重力；`/simulation/ground_truth/odometry` 仅可用于验证和标定，不能接入正式算法闭环。

## 目录

- `worlds/`：平地、台阶、岩石、迷宫和洞穴场景；
- `models/`：地面、岩石、洞穴墙体、天花板与地形模型；
- `config/`：基础/传感器 bridge 与控制器参数；
- `scripts/`：Mid-360 适配器、世界标记与世界生成工具；
- `launch/`：Gazebo 启动入口。
