# Rover_1 历史 CAD 模型

本目录保存 CAD 转换产生的网格、URDF、关节名称表和旧版 launch 文件，用于模型对照与后续迁移；`COLCON_IGNORE` 表明它不参与当前 ROS 2 工作区构建。

- `meshes/`：CAD 导出的 STL 网格；
- `urdf/`：转换后的 URDF/Xacro 与关节表；
- `config/`：关节名称映射；
- `launch/`：旧版显示与 Gazebo 入口。

当前工程使用同级 `Rover_0`。不得直接在运行环境中同时加载两个版本。
