# lunar_rover_description

当前主机器人描述包。`urdf/lunar_rover.urdf.xacro` 定义车体、六轮摇臂机构、传感器外参、五自由度机械臂及控制关节名。

```bash
ros2 launch lunar_rover_description display.launch.py rviz:=true
```

修改 URDF 时必须同步校验：关节名称、`map -> odom -> base_link` 外的传感器 frame、控制器配置和 [`../../../docs/interface_contract.md`](../../../docs/interface_contract.md) 中的接口约定。`Rover_1` 是未纳入当前构建的历史 CAD 转换版本。

`urdf/` 保存主模型和历史备份，`launch/` 是模型显示入口，`rviz/` 保存模型与深度点云视图，`CMakeLists.txt`、`package.xml` 定义 ROS 2 包安装与依赖。
