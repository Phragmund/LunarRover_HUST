# lunar_rover_sensors

原始传感器入口包。该包只负责把仿真或实车传感器暴露为统一 ROS 话题，不做定位、滤波、语义识别或导航决策。

## Launch

| 文件 | 说明 |
| --- | --- |
| `launch/sim_sensors.launch.py` | 仿真模式下把 `/camera/depth/image_raw` 和 `/camera/depth/camera_info` 转为 `/camera/depth_registered/points`，雷达和 IMU 原始桥接由 simulation 包完成。 |
| `launch/real_sensors.launch.py` | 实车模式下启动 Livox ROS driver 和 Orbbec Gemini 335，并统一话题名。 |

## 配置

| 文件 | 说明 |
| --- | --- |
| `config/livox_mid360.json` | Livox Mid-360 实车驱动配置。 |
| `config/gemini335_real.yaml` | Orbbec Gemini 335 实车相机参数。 |

## 主要话题

- `/livox/points`：Mid-360 点云。
- `/livox/imu`：Mid-360 IMU。
- `/camera/color/image_raw`：彩色图。
- `/camera/depth/image_raw`：深度图。
- `/camera/depth_registered/points`：深度图转换出的观察点云。
