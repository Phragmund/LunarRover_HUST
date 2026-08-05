# lunar_rover_description

机器人描述包，提供六轮摇臂月球车模型和模型可视化配置。

## 主要文件

| 路径 | 说明 |
| --- | --- |
| `urdf/lunar_rover.urdf.xacro` | 当前主模型，包含车体、六轮、四个转向关节、摇臂/副摇臂、传感器外参和仿真控制接口。 |
| `urdf/lunar_rover_original.urdf.xacro` | 历史模型备份，用于对比。 |
| `launch/display.launch.py` | 不启动 Gazebo，仅用 `robot_state_publisher`、`joint_state_publisher_gui` 和 RViz 查看模型。 |
| `rviz/lunar_rover.rviz` | 模型和原始传感器查看配置。 |
| `rviz/lunar_rover_depth_points.rviz` | 深度点云观察配置。 |

## 使用

```bash
ros2 launch lunar_rover_description display.launch.py rviz:=true
```

修改模型时需要同步检查传感器 frame、控制关节名和下位机接口文档，避免破坏 `lunar_rover_control` 与实车 `/joint_states` 契约。
