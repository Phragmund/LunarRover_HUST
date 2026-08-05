# lunar_rover_mapping

建图与里程计实验包，当前主线是 FAST-LIO2 注册点云生成 2.5D traversability 地图，同时保留组合轮速 IMU 里程计和旧 2D/OctoMap 调试入口。

## 当前主线

```text
/localization/cloud_registered
  -> terrain_mapper_node.py
  -> /terrain/traversability
  -> lunar_rover_navigation/TerrainLayer
```

组合里程计链路：

```text
/joint_states + /livox/imu
  -> composite_wheel_imu_odometry_node.py
  -> /localization/composite_odometry
```

## 文件

| 路径 | 说明 |
| --- | --- |
| `launch/terrain_mapping.launch.py` | 只启动 FAST-LIO2-backed terrain mapper。 |
| `launch/mapping.launch.py` | 旧 2D mapping 调试入口，包含 slam_toolbox、OctoMap/elevation grid 等。 |
| `launch/sim_mapping.launch.py` | 仿真 + 旧 mapping 调试组合入口。 |
| `scripts/terrain_mapper_node.py` | 点云栅格化，计算高度、坡度、粗糙度、台阶和通行代价。 |
| `scripts/composite_wheel_imu_odometry_node.py` | 轮组前向距离 + IMU 航向的组合平面里程计。 |
| `scripts/imu_ekf_preprocessor.py` | 旧 EKF 方案的 IMU 预处理。 |
| `scripts/imu_dead_reckoning_node.py` | IMU dead reckoning 实验节点。 |
| `scripts/odometry_path_compare.py` | 里程计路径对比和评估辅助节点。 |
| `src/elevation_grid_node.cpp` | C++ elevation grid 调试节点。 |
| `config/terrain_mapper.yaml` | 当前 terrain mapper 主参数。 |
| `config/composite_odometry.yaml` | 组合里程计参数。 |
| `rviz/mapping.rviz` | 定位、地形和探索联调用 RViz 配置。 |

详见 [`../../docs/composite_wheel_imu_odometry.md`](../../docs/composite_wheel_imu_odometry.md)。
