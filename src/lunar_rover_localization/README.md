# lunar_rover_localization

FAST-LIO2 适配与 TF 桥接包。该包让外部 FAST-LIO2 后端输出适合本项目导航链路使用的定位、点云和 `map -> odom` 关系。

## 主要链路

```text
/livox/points + /livox/imu
  -> FAST-LIO2
  -> /localization/lio_odometry_raw
  -> fast_lio2_map_frame_node.py (fast_lio2_map_frame, realtime odom only)
  -> /localization/lio_odometry
  -> map_odom_tf_bridge.py
  -> map -> odom -> base_link
```

`fast_lio2_map_frame_node.py` 会以两个独立进程启动：

- `fast_lio2_map_frame`：只订阅 raw odom 和必要 IMU/TF，发布 `/localization/lio_odometry`，供 `map_odom_tf_bridge.py` 和 Nav2 使用。
- `fast_lio2_map_frame_visualization`：发布 `/localization/cloud_registered`、`/localization/lio_map_cloud` 和 `/localization/lio_path`，这些可视化/建图输出做低频限流，不阻塞实时 odom 转发。

## 文件

| 路径 | 说明 |
| --- | --- |
| `launch/fast_lio2_localization.launch.py` | 启动 FAST-LIO2 后端、输入适配、map frame 校正和 TF 桥。 |
| `scripts/fast_lio2_map_frame_node.py` | 对 FAST-LIO2 原始 odom、path、注册点云和地图点云做 frame 归一化与固定水平补正；启动时拆成实时 odom 实例和低频可视化实例。 |
| `scripts/map_odom_tf_bridge.py` | 使用全局 LIO 位姿和本地组合里程计计算 `map -> odom`。 |
| `scripts/fast_lio2_input_normalizer.py` | 在需要时归一化 FAST-LIO2 输入话题。 |
| `scripts/sim_fast_lio_imu_adapter.py` | 仿真 IMU 适配调试节点。 |
| `config/fast_lio2_mid360.yaml` | FAST-LIO2 Mid-360 参数。 |
| `config/fast_lio2_map_frame.yaml` | map frame 校正和输出话题参数。 |
| `config/map_odom_tf_bridge.yaml` | `map -> odom` 发布参数。 |

## 注意

当前默认配置使用固定补正，不再依赖启动静置采样：

- IMU 等效安装 pitch：`35.273 deg`
- FAST-LIO map 水平补正：`roll=-0.58 deg, pitch=-0.67 deg, yaw=+0.007 deg`

完整系统应优先使用根目录的门控脚本，等待 `/localization/lio_odometry` 可用后再启动地形建图和控制链路。
