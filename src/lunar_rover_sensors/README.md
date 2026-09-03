# lunar_rover_sensors

统一 Livox Mid-360 与 Gemini 335 的仿真/实车原始数据入口；不执行定位、语义识别或导航决策。

| 接口 | 类型 |
| --- | --- |
| `/livox/points` | `sensor_msgs/msg/PointCloud2` |
| `/livox/imu` | `sensor_msgs/msg/Imu` |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` |
| `/camera/depth_registered/points` | `sensor_msgs/msg/PointCloud2` |

`sim_sensors.launch.py` 提供仿真深度点云转换；`real_sensors.launch.py` 启动实车驱动。设备权限、IP 与序列号均为部署配置，不应硬编码为公共接口。

`config/` 存放 Livox 与 Gemini 参数，`launch/` 提供仿真、实车和相机单独测试入口，`rviz/` 提供 Gemini 可视化配置。`gemini335_rtabmap.launch.py` 是独立 RGB-D 实验入口，不属于当前 FAST-LIO2 主链路。
