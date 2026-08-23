# lunar_rover_sensors

原始传感器入口包。该包只负责把仿真或实车传感器暴露为统一 ROS 话题，不做定位、滤波、语义识别或导航决策。

> 📄 **Gemini 335L 相机的完整接口与使用说明见 [`docs/gemini335_camera.md`](../../docs/gemini335_camera.md)**（硬件、话题、配置、启动、验证、踩坑、与雷达融合标定）。

## Launch

| 文件 | 说明 |
| --- | --- |
| `launch/sim_sensors.launch.py` | 仿真模式下把 `/camera/depth/image_raw` 和 `/camera/depth/camera_info` 转为 `/camera/depth_registered/points`，雷达和 IMU 原始桥接由 simulation 包完成。 |
| `launch/real_sensors.launch.py` | 实车模式下启动 Livox ROS driver 和 Orbbec Gemini 335，并统一话题名。 |
| `launch/gemini335_test.launch.py` | 仅启动 Orbbec Gemini 335 相机（不含 Livox），并可选打开 RViz，用于开箱/功能测试。 |

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
- `/camera/depth/points`：深度点云（真实相机由 orbbec 驱动原生发布）。
- `/camera/depth_registered/points`：仿真下由 depth_image_proc 从深度图转换出的观察点云。

## Gemini 335L 开箱 / 功能测试

真实相机单独测试（不启动 Livox / 底盘），验证 RGB、深度、点云三路数据。

### 一次性准备：安装 udev 规则

非 root 用户访问 Orbbec 相机需要 udev 规则（含 `2bc5:0804` = Gemini 335L）：

```bash
sudo cp "$(dirname "$(readlink -f "$0")")/../install/orbbec_camera/share/orbbec_camera/udev/99-obsensor-libusb.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

> 更简单：直接复制仓库 `install/orbbec_camera/share/orbbec_camera/udev/99-obsensor-libusb.rules` 到 `/etc/udev/rules.d/` 后 reload。

### 环境 / 设备自检

```bash
./scripts/gemini335_check.sh          # 静态检查 + USB 检测
./scripts/gemini335_check.sh --live   # 额外启动驱动，实测话题与帧率
```

### 即插即测（相机 + RViz）

```bash
./scripts/gemini335_test.sh            # RGB 彩图 + 深度灰度图 + 深度点云（RViz）
./scripts/gemini335_test.sh --no-rviz  # 只启动驱动，不弹 RViz
```

RViz 配置：`rviz/gemini335_test.rviz`，固定坐标系 `gemini335_color_optical_frame`，三个显示面板分别订阅：

- RGB 彩色画面 —— `/camera/color/image_raw`
- 深度灰度图 —— `/camera/depth/image_raw`（16UC1，Normalize Range 归一化显示）
- 深度点云 —— `/camera/depth/points`（PointCloud2）

> 说明：`config/gemini335_real.yaml` 中 color/depth/point_cloud 的 QoS 均为 `SENSOR_DATA`（Best Effort），RViz 三个显示也需对应 Best Effort，否则收不到数据。
