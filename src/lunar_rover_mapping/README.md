# lunar_rover_mapping

建图与里程计实验包，当前主线是 FAST-LIO2 注册点云生成基础地形高度图，同时保留组合轮速 IMU 里程计和旧 2D/OctoMap 调试入口。

## 当前主线

```text
/localization/cloud_registered
  -> terrain_mapper_node.py
  -> /terrain/traversability
```

`/terrain/traversability` 当前保留旧话题名以兼容 RViz 配置，但语义已经改为高度图可视化，不再是可直接给 Nav2 使用的通行代价图：

| 值 | 语义 |
| --- | --- |
| `-1` | 未知或没有可用点云。 |
| `5-55` | 地面高度色阶，越高表示低处地面簇的高度越高。当前默认把 `0.90-1.10 m` 映射为完整色阶，两端与该高度范围边界重合。 |
| `100` | 墙体：同一 XY 投影格内，点云在较大 Z 范围内连续/均匀分布。 |

分类规则是先对每个 XY 栅格做 Z 向直方图。当前 `min_points_per_cell=1`，也就是单个有效点即可让该 5cm 栅格进入高度计算。如果高度上连续占据较多 bin，判为墙体；如果只在低处和高处形成两个簇，中间基本空缺，则取低处簇作为地面，高处簇视为天花板并忽略。

RViz 默认使用 `lunar_rover_mapping/RainbowOccupancyGrid` 插件显示这个话题。该插件仍然订阅 `nav_msgs/OccupancyGrid`，但不使用 RViz 内置 `Map` 的灰度/代价色表，而是把二维栅格直接画成贴地彩虹像素图层：`5-55` 映射到地面高度色阶，`100` 固定为墙体颜色，未知区域透明。该图层关闭深度测试，按二维地图叠加显示，避免被点云或世界模型遮挡。旧的 `/terrain/height_markers` 体素彩虹层仅保留为调试出口，默认不启用。

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
| `scripts/terrain_mapper_node.py` | 点云栅格化，按 Z 向分布区分墙体、地面和天花板，并发布地面高度色阶图。 |
| `scripts/composite_wheel_imu_odometry_node.py` | 轮组前向距离 + IMU 航向的组合平面里程计。 |
| `scripts/imu_ekf_preprocessor.py` | 旧 EKF 方案的 IMU 预处理。 |
| `scripts/imu_dead_reckoning_node.py` | IMU dead reckoning 实验节点。 |
| `scripts/odometry_path_compare.py` | 里程计路径对比和评估辅助节点。 |
| `src/elevation_grid_node.cpp` | C++ elevation grid 调试节点。 |
| `src/rainbow_occupancy_grid_display.cpp` | 自定义 RViz 二维彩虹高度图显示插件。 |
| `config/terrain_mapper.yaml` | 当前 terrain mapper 主参数。 |
| `config/composite_odometry.yaml` | 组合里程计参数。 |
| `rviz/mapping.rviz` | 定位、地形和探索联调用 RViz 配置。 |

详见 [`../../docs/composite_wheel_imu_odometry.md`](../../docs/composite_wheel_imu_odometry.md)。
