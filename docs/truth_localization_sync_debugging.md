# Truth Localization Synchronization Debugging

本文记录仿真真值替换模式下，`/terrain/traversability` 在快速转弯时出现扇形发散的根因和修复方法。

## 适用场景

启动方式：

```bash
TRUTH_LOCALIZATION_MODE=replace_fastlio scripts/restart_gated_mapping_exploration_navigation.sh
```

该模式下 `truth_localization_node.py` 用 Gazebo 真值位姿替代 FAST-LIO：

```text
/simulation/ground_truth/odometry + /livox/points
  -> /localization/lio_odometry
  -> /localization/cloud_registered
```

这里的真值只提供车体位姿；点云仍来自仿真 LiDAR。`truth_localization` 的职责是把每一帧 LiDAR 点云按对应车体位姿投到 `map` 坐标系。

## 问题症状

快速原地转弯或急转时，RViz 中的 traversability/costmap 可能出现扇形、射线状或墙体外扩的发散。典型表现：

- 慢速直行看起来正常；
- 快速转向时墙边出现紫色/黑色长条；
- 点云或 traversability 像被扫帚扫开；
- 有时日志出现 `extrapolation into the future` 或 `raytracing disabled for this cloud`。

## 根因

这次实际遇到的是两层同步问题。

第一层是 `terrain_mapper_node.py` 的 TF 查询太严格。`/localization/cloud_registered` 已经是 `map` frame 点云，但 terrain mapper 仍会按点云 stamp 查询 `map->base_link` 来做机器人自滤波和 raytracing 起点计算。DDS 到达顺序可能让点云先于同 stamp 的 TF 进入 terrain mapper，导致：

```text
Lookup would require extrapolation into the future
Cannot locate ray origin base_link in map; raytracing disabled for this cloud
```

修复方法：terrain mapper 对 TF 查询使用 latest fallback，并且输入云已经是 `map` frame 时直接使用单位变换。这样启动或传输顺序抖动不会关闭 raytracing。

第二层是真正导致快速转向扇形发散的原因。`truth_localization_node.py` 早期实现用 `latest_truth` 摆放每一帧 LiDAR 点云。快速转弯时，LiDAR 帧时间戳和“最新 truth odom”时间戳不同，几十到几百毫秒的姿态差就会把点云投影成扇形。

修复方法：`truth_localization` 维护 ground truth pose buffer，按 `/livox/points.header.stamp` 查找对应真值位姿；相邻 truth pose 之间用位置线性插值和 quaternion slerp。找不到时间对齐位姿时丢弃该帧点云，不再用错位姿态硬投影。

## 修复位置

关键文件：

```text
src/lunar_rover_localization/scripts/truth_localization_node.py
src/lunar_rover_mapping/scripts/terrain_mapper_node.py
```

`truth_localization_node.py` 的关键点：

- 缓存 `/simulation/ground_truth/odometry`；
- 按 LiDAR stamp 查询/插值 truth pose；
- 使用匹配到的 truth pose 注册点云；
- `/localization/cloud_registered.header.stamp` 使用匹配 pose 的 stamp；
- 没有时间对齐 pose 时记录 `Dropping LiDAR cloud...` 并丢弃。

`terrain_mapper_node.py` 的关键点：

- `lookup_transform_with_latest_fallback()`；
- 同 frame 时返回 identity transform；
- TF 按 stamp 查不到时 fallback 到 latest；
- 避免 raytracing 因短暂 TF 传输顺序问题被禁用。

## 验证方法

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select lunar_rover_localization lunar_rover_mapping
```

干净重启：

```bash
TRUTH_LOCALIZATION_MODE=replace_fastlio scripts/restart_gated_mapping_exploration_navigation.sh
```

检查 ROS graph：

```bash
ros2 node list | sort | rg 'truth|terrain|elevation|fast|composite'
ros2 topic hz /localization/cloud_registered
ros2 topic echo --once /terrain/diagnostics
```

期望状态：

```text
/truth_localization running
/terrain_mapper running
/fastlio_mapping not running
/localization/cloud_registered about 8-9 Hz
terrain diagnostics: last_cloud=yes
```

检查日志：

```bash
rg -n "Dropping LiDAR|extrapolation|raytracing disabled|terrain_mapper.*Cannot" /tmp/lunar_rover_*.log
```

期望结果：

- 启动初期偶发 `livox_frame` 或 `base_link` 未进入 TF buffer 的警告可以接受；
- 运行中不应持续出现 `extrapolation into the future`；
- 运行中不应持续出现 `raytracing disabled for this cloud`；
- 快速转弯时 RViz 中不应再出现扇形发散。

## 后续排查顺序

如果后续再次出现类似发散，按这个顺序查：

1. 确认当前是不是 `TRUTH_LOCALIZATION_MODE=replace_fastlio`，不要和 FAST-LIO 模式混淆。
2. 查 `/localization/cloud_registered` 是否由 `truth_localization` 发布。
3. 查日志是否有持续 `Dropping LiDAR cloud`。如果有，说明 LiDAR stamp 超出 truth buffer 或时间源不一致。
4. 查日志是否有持续 `extrapolation into the future`。如果有，说明 terrain mapper 或其他下游节点又在做严格 stamp TF 查询。
5. 查地面是否接近 `map z=0`，以及 `map->base_link.z` 是否接近 URDF 标称离地高度 `0.1825 m`。真值模式必须用车辆落稳后的 `/simulation/ground_truth/odometry.pose.pose.position.z - TRUTH_BASE_HEIGHT_ABOVE_GROUND` 作为 `map_origin_world_z`，不能直接用 `SPAWN_Z` 或 base_link 高度。否则 RViz 车体会偏低或偏高，点云/terrain 相对车体高度异常。
6. 如果以上都正常但仍发散，再考虑 LiDAR 单帧运动畸变、raytracing 策略或点云输入本身。

验证高度基准：

```bash
ros2 topic echo --once /simulation/ground_truth/odometry --field pose.pose.position.z
ros2 topic echo --once /localization/lio_odometry --field pose.pose.position
ros2 run tf2_ros tf2_echo map base_link
```

期望 `/localization/lio_odometry.pose.pose.position.z` 和 `map->base_link` 的 z 接近 `0.1825 m`，点云地面低分位接近 `0`。脚本日志中应出现类似：

```text
truth mode map_origin_z adjusted to settled ground z=... (base_z=... clearance=0.1825)
```
