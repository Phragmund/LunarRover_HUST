# Orbbec Gemini 335L 深度相机 —— 接口与使用说明

> 适用范围：本项目（LunarRover，ROS 2 Humble）的 Orbbec Gemini 335L 双目结构光深度相机。
> 本文档记录已验证可用的硬件接口、ROS 话题、配置、启动方式与常见问题，供后续维护/交接人员复刻操作。

---

## 1. 硬件信息

| 项目 | 值 |
| --- | --- |
| 型号 | Orbbec Gemini 335L（双目结构光深度相机，RGB-D） |
| USB VID:PID | `2bc5:0804`（`lsusb` 显示为 "Orbbec Gemini 335L"） |
| 序列号 | `CP2L8530004T` |
| 固件版本 | `1.4.60` |
| ISP 固件 | `20241125`（与 SDK 要求一致，无需升级） |
| SDK / ROS Wrapper | `2.9.3` / `2.9.3` |
| 连接方式 | USB（推荐 USB3 蓝色口 + USB3 数据线；USB2 也能用但点云/彩色帧率会下降） |

> 备注：实测走 USB2.1 时功能正常，但彩色流（MJPG 解码）与点云帧率偏低。正式使用建议 USB3。

---

## 2. 环境依赖

| 依赖 | 说明 |
| --- | --- |
| ROS 2 | Humble（`/opt/ros/humble`） |
| Orbbec 驱动 | `orbbec_camera`（源码 `src/third_party/OrbbecSDK_ROS2`，已构建到工作区 `install/`） |
| 依赖包 | `orbbec_camera_msgs`、`orbbec_description`、`image_transport`、`depth_image_proc`、`tf2` |
| 构建 | 工作区使用 `--symlink-install`（`scripts/build.sh` 统一构建） |

验证驱动/包是否就绪：

```bash
source /opt/ros/humble/setup.bash
source <项目根>/install/setup.bash
ros2 pkg list | grep -E "orbbec_camera|lunar_rover_sensors"
```

---

## 3. 一次性准备：udev 规则

非 root 用户访问 Orbbec 相机需要 udev 规则（含 `2bc5:0804` = Gemini 335L）。只装一次：

```bash
sudo cp <项目根>/install/orbbec_camera/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

验证：插入相机后应出现符号链接 `/dev/Gemini_335L -> bus/usb/...`。

---

## 4. 坐标系与 TF

相机驱动默认的帧命名（`camera_name=camera`）：

| 帧名 | 含义 |
| --- | --- |
| `camera_link` | 相机底座（传感器基座） |
| `camera_color_frame` / `camera_color_optical_frame` | 彩色相机系 / 彩色光心系 |
| `camera_depth_frame` / `camera_depth_optical_frame` | 深度相机系 / 深度光心系 |

- 深度配准开启（`depth_registration: true`）时，深度图与彩色图共用 `camera_color_optical_frame`（深度已对齐到彩色）。
- 未配准的深度图（`/camera/depth/image_unaligned`）使用 `camera_depth_optical_frame`。
- 点云帧由 `cloud_frame_id` 指定（实车/测试配置为 `gemini335_color_optical_frame`；该名在 URDF 中通过 alias joint 与 `camera_color_optical_frame` 关联）。
- 是否发布 TF 由 `publish_tf` 控制：实车/测试为 `false`（不抢 URDF 静态 TF），建图/标定为 `true`（发布 `camera_link -> ...` 静态 TF）。

---

## 5. ROS 话题接口

命名空间 `camera`（`camera_name:=camera`）。以下为 `gemini335_test.yaml` 配置下的实测结果（640×400 @ 15fps，`depth_registration: true`）：

| 话题 | 类型 | 内容 / 编码 | 帧 | QoS（test 配置） |
| --- | --- | --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 彩色图，`rgb8`，640×400 | `camera_color_optical_frame` | Reliable（`default`） |
| `/camera/color/image_raw/compressed` | `CompressedImage` | MJPG 压缩流 | — | — |
| `/camera/color/camera_info` | `CameraInfo` | 彩色内参 K + 畸变 | `camera_color_optical_frame` | Reliable |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图，`16UC1`，单位毫米(mm)，640×400 | `camera_color_optical_frame`（已配准） | Reliable（`default`） |
| `/camera/depth/image_raw/compressedDepth` | `CompressedImage` | 深度压缩流 | — | — |
| `/camera/depth/image_unaligned` | `sensor_msgs/Image` | 未配准深度（原始深度系） | `camera_depth_optical_frame` | — |
| `/camera/depth/camera_info` | `CameraInfo` | 深度内参 | `camera_depth_optical_frame` | Reliable |
| `/camera/depth/points` | `PointCloud2` | 深度点云，XYZ（`point_step=16`），单位米(m) | `gemini335_color_optical_frame` | Best Effort（`SENSOR_DATA`） |
| `/camera/depth_filters/status` | — | 深度滤波状态 | — | — |
| `/camera/device_status` | — | 设备状态 | — | — |
| `/camera/color/metadata`、`/camera/depth/metadata` | — | 元数据 | — | — |

单位关键点：深度图是 mm（如 5561 = 5.56m），点云是 m。做 RTAB-Map 等消费方时需 `depth_scale=0.001`。

---

## 6. 配置文件（`src/lunar_rover_sensors/config/`）

| 文件 | 用途 | 关键差异 |
| --- | --- | --- |
| `gemini335_real.yaml` | 实车（`real_sensors.launch.py`） | `publish_tf: false`；color/depth/point_cloud QoS 全 `SENSOR_DATA`（Best Effort） |
| `gemini335_test.yaml` | 单独测试 | 同 real，但 color/depth QoS 改 `default`（Reliable），修 RViz 图像不显示 |
| `gemini335_mapping.yaml` | RTAB-Map 建图/标定 | `publish_tf: true`；`enable_point_cloud: false`；color/depth QoS `default` |

三者共同的 640×400@15 关键参数：

```yaml
depth_registration: true      # 深度配准到彩色
enable_frame_sync: true       # 硬件帧同步
enable_sync_host_time: true
color_width: 640  color_height: 400  color_fps: 15
depth_width: 640  depth_height: 400  depth_fps: 15
```

---

## 7. 启动方式

### 7.1 实车整包（相机 + 雷达）

```bash
ros2 launch lunar_rover_sensors real_sensors.launch.py
```

同时启动 Livox Mid-360 驱动与 Orbbec 相机，并把点云重映射为 `/camera/depth_registered/points`。

### 7.2 单独测试相机（RGB + 深度 + 点云 + RViz）

```bash
# 推荐：一键脚本（含 USB/udev 检查）
./scripts/gemini335_test.sh            # 相机 + RViz
./scripts/gemini335_test.sh --no-rviz  # 只启动相机

# 或直接 launch
ros2 launch lunar_rover_sensors gemini335_test.launch.py
ros2 launch lunar_rover_sensors gemini335_test.launch.py start_rviz:=false
```

### 7.3 环境/设备自检

```bash
./scripts/gemini335_check.sh           # 静态检查 + USB 检测（不启动）
./scripts/gemini335_check.sh --live    # 额外启动驱动，实测话题与帧率
```

### 7.4 深度相机建图（RTAB-Map，实验性，非主力）

```bash
ros2 launch lunar_rover_sensors gemini335_rtabmap.launch.py
```

管线：相机(`publish_tf`) → `rgbd_sync` → `rgbd_odometry`(视觉里程计) → `rtabmap` → `rtabmap_viz`。
> 注：本项目正式路线是「相机辅助雷达建图」，相机单独建图仅作调试/验证，不作为主力。

---

## 8. RViz 可视化

配置文件：`src/lunar_rover_sensors/rviz/gemini335_test.rviz`（已随包安装）。

三个显示面板：

| 面板 | 话题 | 说明 |
| --- | --- | --- |
| RGB Color Image | `/camera/color/image_raw` | 彩色画面 |
| Depth Grayscale Image | `/camera/depth/image_raw` | 深度灰度（16UC1，开启 Normalize Range 归一化） |
| Depth Point Cloud | `/camera/depth/points` | PointCloud2，固定坐标系 `gemini335_color_optical_frame` |

---

## 9. 手动验证命令（无 GUI 也能验）

```bash
source /opt/ros/humble/setup.bash
source <项目根>/install/setup.bash

# 话题列表
ros2 topic list | grep '^/camera/'

# 帧率（深度应 ~15fps）
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/color/image_raw

# 拿一帧看编码/尺寸
ros2 topic echo /camera/color/image_raw --once | grep -E 'encoding|width|height|frame_id'
ros2 topic echo /camera/depth/image_raw --once | grep -E 'encoding|width|height'
ros2 topic echo /camera/depth/points  --once | grep -E 'frame_id|width|height|point_step'

# 查 QoS（确认发布端可靠性）
ros2 topic info /camera/color/image_raw -v
```

判断相机功能完好的标准（实测值）：

- 彩色：`rgb8` 640×400，有稳定输出；
- 深度：`16UC1` 640×400，帧率 ≈15fps，非零像素合理（毫米值，如 300~6000）；
- 点云：`PointCloud2`，帧 `gemini335_color_optical_frame`，点数约十几万。

---

## 10. 常见问题 / 踩坑记录

### 10.1 RViz 图像面板空白，但话题在发布

RViz（Humble）的 Image 显示在 Best Effort QoS 下不建立订阅（点云显示不受影响）。解决：图像用 Reliable QoS（`gemini335_test.yaml` 已把 `color_qos/depth_qos` 改为 `default`），RViz 图像面板 Reliability Policy 也设为 Reliable。

### 10.2 点云帧率远低于深度帧率

点云单帧约 3MB，Best Effort 下 `ros2 topic hz`/`echo` 会因队列小丢消息，测出偏低是测量假象。用 RViz 或加大订阅队列看真实速率。

### 10.3 驱动日志出现 openUsbDevice failed! status:113

这是设备复位/重连的瞬态，驱动会自动重试（约 10s 一次），几十秒内恢复 `Device ... connected`，无需干预。

### 10.4 USB2 vs USB3

插 USB2 口时驱动日志显示 `usb connect type: USB2.1`，功能正常但帧率/带宽受限。满帧率请用 USB3 口 + USB3 线。

### 10.5 帧名不一致

点云帧是 `gemini335_color_optical_frame`（`cloud_frame_id`），彩色/深度图帧是 `camera_color_optical_frame`。两者在 URDF 里是 alias 关系；脱离 URDF 单独用时，注意 RViz 固定坐标系要对应点云帧。

---

## 11. 与雷达融合（相机补地面障碍细节）

- 融合节点：`lunar_rover_mapping/scripts/multisensor_cloud_fusion_node.py`（配置 `config/cloud_fusion.yaml`），已实现「LiDAR 锚定 + 相机补细节」，用 TF 把相机点云与雷达点云变换到 `map` 帧。
- 相机点云来源：`/camera/depth_registered/points`（实车），雷达：`/localization/cloud_registered`。
- 外参标定（相机↔雷达 6-DOF）决定融合精度。推荐靶标法（棋盘格标定板），工具：Livox 官方 `livox_camera_calib`、Autoware `CalibrationTools`、`FAST-Calib_ROS2`。
- 标定板可自制：A4 打印棋盘格拼到 60cm 左右、平整坚硬的 KT板/雪弗板上，格子约 8cm。关键是平、刚、够大。
- 精度目标：角度 <0.3°、平移 <2cm 即可满足近地面补细节。

---

## 12. 关键文件清单

| 路径 | 作用 |
| --- | --- |
| `src/lunar_rover_sensors/config/gemini335_real.yaml` | 实车相机参数 |
| `src/lunar_rover_sensors/config/gemini335_test.yaml` | 单独测试参数（图像 Reliable） |
| `src/lunar_rover_sensors/config/gemini335_mapping.yaml` | 建图/标定参数（publish_tf） |
| `src/lunar_rover_sensors/launch/real_sensors.launch.py` | 实车：雷达 + 相机 |
| `src/lunar_rover_sensors/launch/gemini335_test.launch.py` | 相机单独测试 + RViz |
| `src/lunar_rover_sensors/launch/gemini335_rtabmap.launch.py` | 相机 RTAB-Map 建图 |
| `src/lunar_rover_sensors/rviz/gemini335_test.rviz` | RGB + 深度 + 点云视图 |
| `scripts/gemini335_test.sh` | 即插即测一键脚本 |
| `scripts/gemini335_check.sh` | 环境/设备自检脚本 |
| `src/lunar_rover_sensors/README.md` | 包内简要说明 |

---

## 附录：快速复刻流程（新环境）

```bash
# 1. 构建（首次）
./scripts/build.sh

# 2. 一次性：udev 规则（见第 3 节）

# 3. 插相机（USB3），自检
./scripts/gemini335_check.sh --live

# 4. 即插即测（RGB + 深度 + 点云 + RViz）
./scripts/gemini335_test.sh
```

