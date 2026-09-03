# lunar_rover_description

机器人模型目录，包含两个来源不同的描述版本。

| 子目录 | 状态 | 内容 |
| --- | --- | --- |
| `Rover_0/` | 当前使用 | ROS 2 包 `lunar_rover_description`；包含主 Xacro、RViz 配置和模型显示 launch。 |
| `Rover_1/` | 历史保留 | 从 CAD 转换的 URDF、网格和旧 Gazebo/RViz 启动文件；通过 `COLCON_IGNORE` 排除在当前构建外。 |

所有运行链路仅使用 `Rover_0`。若启用或替换 `Rover_1`，须先统一包名、关节名、frame、控制器配置和传感器外参，不能与当前主模型混用。
