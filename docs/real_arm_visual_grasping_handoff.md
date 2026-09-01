# 实车机械臂视觉抓取链路交接

更新时间：2026-08-29 16:35 CST

本文记录当前实车阶段已经完成和验证过的内容，重点服务后续接手继续调试。总体技术路线参见
`docs/月球熔岩管移动操作机器人_视觉搜索抓取与存放技术方案.md`，目标识别与
FoundationPose 设计参见 `docs/target_perception_and_foundationpose.md`。

## 当前状态概览

已经打通：

- Gemini 335L RGB-D 相机 ROS2 链路。
- 机械臂 H7 串口控制链路，ROS 侧可用 topic 方式发送 `$ARM:MULTI`。
- ROS 侧机械臂 IK 键盘控制器。
- ChArUco 手眼采样、初步手眼求解、RViz 临时 TF 预览。
- HSV 高饱和目标检测、mask 输出、RGB-D 三维位置解算。
- RViz 中用简化模型显示机械臂连杆、关节、工具点、相机和识别到的物块。

当前没有完成：

- 手眼外参还没有写入正式 URDF/launch，仅由临时预览节点发布。
- `grasp_coordinator` 的机械臂几何参数仍是旧值，尚未同步最新实测臂长。
- YOLOWorld / SAM 在当前系统 Python 下没有加载成功，实际运行仍是 HSV-only。
- 自动抓取动作尚未实机闭环验证。
- 手眼精度目前是初步可视化级别，残差约 20 mm，不应直接视为最终标定。

## 实车硬件与基础链路

当前使用的关键硬件：

- 下位机：H7 综合控制板。
- 机械臂：5 个舵机通道，`arm_joint1` 到 `arm_joint5`，其中 `arm_joint5` 是夹爪。
- 相机：Orbbec Gemini 335L。
- 本阶段暂时不要求底盘导航运动，只做视觉抓取验证。

低日志机械臂控制启动方式：

```bash
source install/setup.bash
ros2 run lunar_rover_control motion_solver.py --ros-args \
  --params-file install/lunar_rover_control/share/lunar_rover_control/config/arm_hardware.yaml \
  -p use_sim_time:=false \
  -p h7_port:=/dev/ttyACM0 \
  -p arm_read_period:=0.50 \
  --log-level motion_solver:=warn
```

这样做的原因：

- 默认 `motion_solver` 会每 0.10 s 发送一次 `$ARM:READ`，且每条 `$ARM:RESP` 都打 INFO 日志。
- 实测时日志堆到几十万行后，`/joint_states` 从代码预期的 20 Hz 降到约 1.5 Hz。
- 降低日志级别并将 `arm_read_period` 放慢到 0.50 s 后，`/joint_states` 恢复到约 18-19 Hz。

机械臂脱力：

```bash
source install/setup.bash
ros2 topic pub --once /manual_control lunar_rover_msgs/msg/ManualControl \
  '{lx: 0, ly: 0, rx: 0, ry: 0, relax: true, rearm: false, boost: false, calibrate: false}'
```

该命令通过 `motion_solver` 触发：

```text
$M3508:STOP!
$STOP!
$ARM:TORQUE,1,0!
...
$ARM:TORQUE,5,0!
```

## 机械臂 ROS 端 IK 控制

实现文件：

- `src/lunar_rover_control/scripts/keyboard_arm_ik_controller.py`
- 文档：`src/lunar_rover_control/README.md`

启动：

```bash
source install/setup.bash
ros2 run lunar_rover_control keyboard_arm_ik_controller.py
```

控制方式：

| 按键 | 功能 |
| --- | --- |
| `W` / `S` | 末端目标前 / 后平移 |
| `A` / `D` | 末端目标左 / 右平移 |
| `Space` / `Shift` | 末端目标上 / 下平移 |
| `Up` / `Down` | 末端姿态 pitch 增 / 减 |
| `Right` / `Left` | 夹爪打开 / 闭合 |
| `R` | 回到 IK 初始位 |

坐标定义：

| 轴 | 方向 |
| --- | --- |
| `+X` | 车体正前方 |
| `+Y` | 车体左方 |
| `+Z` | 向上 |

实测臂长参数：

| 参数 | 数值 |
| --- | --- |
| `base_to_shoulder_m` | `0.07545` |
| `upper_arm_m` | `0.11391` |
| `forearm_m` | `0.11000` |
| `tool_m` | `0.15601` |

角度定义：

- `J1 = 90 deg` 时朝向车体正前方，`J1 = 0 deg` 时朝向车体正右方。
- `J2 = 180 deg` 时其 link 指向正上方，`J2 = 90 deg` 时指向车体正前方。
- `J3 = 135 deg`、`J4 = 135 deg` 时与上一个 link 共线。
- 初始竖直向上时，增大 `J3/J4` 会让对应 link 向车体前方倾斜，减小则向后倾斜。

2026-08-29 用户手动摆出的朝向桌面的 IK 初始位已保存为：

```text
home_degrees = [96.754752, 201.804458, 183.510522, 110.293936, 45.0]
```

对应 `/joint_states`：

```text
[0.117893, -0.380560, 0.846668, -0.431202, 0.022500]
```

后续启动 IK 控制器时，默认 target 和 `R` 键回零都使用这个朝向桌面的姿态。

已知控制问题：

- 如果 `motion_solver` 大量刷 `$ARM:RESP` / `$ARM:MULTI` INFO 日志，键盘手感会明显卡顿。
- 键盘节点会自动发现多个 `/dev/input/event*`，如果后续出现重复按键或抽搐，应指定单一
  `input_device` 参数，避免多个 event 设备重复上报。
- 当前 `motion_solver` topic 入口不会自动给机械臂重新上力矩；如果先 `relax`，需要通过
  `rearm` 或 `$ARM:TORQUE,id,1!` 恢复位置保持。

## ChArUco 手眼标定

标定板生成脚本：

- `tools/generate_charuco_board.py`

已生成板：

- `generated/calibration/charuco/charuco_a4_7x10_25mm.pdf`
- `generated/calibration/charuco/charuco_a4_7x10_25mm.png`

用户打印并使用的是：

```text
A4 7x10, square = 25 mm, marker = 18 mm, DICT_5X5_1000
```

采样节点：

- `src/lunar_rover_manipulation/scripts/charuco_handeye_sampler_node.py`

启动：

```bash
source install/setup.bash
ros2 run lunar_rover_manipulation charuco_handeye_sampler_node.py
```

采样操作：

- OpenCV 窗口名称：`ChArUco hand-eye sampler`
- 按 `p` 保存一组样本。
- 按 `q` 退出。
- 采样时应让机械臂停稳 1-2 s。

当前采样目录：

```text
generated/calibration/handeye_samples/20260829_152555
```

当前采样数量：

```text
27 groups
```

数据文件：

- `generated/calibration/handeye_samples/20260829_152555/samples.jsonl`
- `generated/calibration/handeye_samples/20260829_152555/sample_001_overlay.png` 到
  `sample_027_overlay.png`

每组样本包含：

- RGB 图像时间戳和 frame。
- 相机内参 `K/D`。
- `camera_T_board`，由 ChArUco PnP 得到。
- 最近一次 `/joint_states`。
- overlay 图像文件名。

## 初步手眼求解结果

结果文件：

```text
generated/calibration/handeye_samples/20260829_152555/handeye_initial_solution.json
```

数据检查：

```text
total samples = 27
filtered samples = 21
robust samples = 17
charuco corners min / median / max = 24 / 48 / 54
```

初筛剔除样本：

```text
1, 2, 3, 15, 16, 26
```

主要原因：

- 图像时间戳和 `/joint_states` 时间差过大。
- 或 ChArUco 角点数量偏低。

最终稳健求解使用样本：

```text
4, 5, 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 22, 23, 24, 25, 27
```

推荐的初步外参：

```text
tool_T_camera
xyz = (-0.0433, -0.0422, +0.0253) m
rpy = (+102.65, -0.75, +88.31) deg
method = PARK
```

残差：

```text
translation residual mean / median / max = 19.5 / 20.0 / 33.4 mm
```

解释：

- 这里的 `tool_frame` 是 IK 控制器使用的工具点坐标系，不是严格 CAD/URDF 中的 `arm_link4`。
- 相机光学坐标系为常见 optical frame 约定：`+Z` 朝前，`+X` 向右，`+Y` 向下。因此
  `tool_T_camera` 中接近 90 度的旋转并不必然表示相机安装偏差很大。
- 20 mm 级残差说明该结果只适合初步可视化和粗定位验证，后续需要再次采样、同步和求解。

当前平均标定板位姿：

```text
base_link -> charuco_board_frame
xyz = (0.594, 0.152, 0.167) m
rpy = (87.60, 1.05, -92.81) deg
```

## 临时 TF 与 RViz 可视化

临时 TF 预览节点：

- `tools/handeye_tf_preview_node.py`

发布：

```text
base_link -> arm_base_link
arm_base_link -> handeye_tool_frame
handeye_tool_frame -> camera_color_optical_frame
camera_color_optical_frame -> gemini335_color_optical_frame
base_link -> charuco_board_frame
```

注意：

- Gemini 点云实际 frame 是 `gemini335_color_optical_frame`。
- 前一次 RViz 不刷新就是因为只发布了 `camera_color_optical_frame`，没有把
  `gemini335_color_optical_frame` 接到 TF 树中，导致 Message Filter 持续丢点云。

启动：

```bash
source install/setup.bash
python3 tools/handeye_tf_preview_node.py
```

机械臂/目标 RViz marker 节点：

- `tools/arm_grasp_rviz_markers_node.py`

发布 topic：

```text
/visualization/arm_grasp_markers
```

显示内容：

- 蓝色粗线：机械臂连杆。
- 黄色球：`J1/base`、`J2`、`J3`、`J4`。
- 绿色球：`tool`。
- 紫色小方块和箭头：标定后的相机和光轴。
- 绿色半透明方块和红色中心点：当前识别到的物块。
- 白色文字：关节名、目标类别和置信度。

启动：

```bash
source install/setup.bash
python3 tools/arm_grasp_rviz_markers_node.py
```

推荐 RViz 配置：

```bash
source install/setup.bash
rviz2 -d generated/calibration/handeye_samples/20260829_152555/arm_target_3d_preview.rviz
```

该配置只看 3D，避免 RGB/Depth 面板遮挡判断。另一个配置
`handeye_preview.rviz` 同时包含 RGB/Depth 图像面板。

已截图确认：

- `Arm Grasp Markers` display 状态为 OK。
- 点云、机械臂、工具点、相机、标定板、目标 marker 可以在 `base_link` 下显示。

## 视觉识别、分割与三维解算

启动视觉抓取验证链路：

```bash
source install/setup.bash
ros2 launch lunar_rover_manipulation real_grasping.launch.py \
  detector_mode:=fusion \
  open_vocab_enabled:=true \
  sam_enabled:=false \
  active_search:=false \
  auto_trigger_grasp:=false \
  periodic_sweep:=false
```

当前实际运行情况：

- `target_fusion_detector_node.py` 成功启动。
- 系统 Python 缺少 `ultralytics`，所以 YOLOWorld 未启用。
- 日志为：

```text
open-vocabulary model unavailable, using HSV only: No module named 'ultralytics'
```

因此当前实际链路是：

```text
/camera/color/image_raw/compressed
  -> HSV high-saturation connected components
  -> /perception/target_mask
  -> /perception/target_detection
/camera/depth/image_raw + mask + TF
  -> target_localizer_node.py
  -> /perception/target_observation
  -> /grasp/target_pose
```

关键话题：

| Topic | 类型/作用 |
| --- | --- |
| `/perception/debug/fusion_overlay` | 普通 debug overlay 图像 |
| `/perception/debug/fusion_overlay/compressed` | 压缩 debug overlay |
| `/perception/target_mask` | 当前目标 mask |
| `/perception/target_detection` | 2D bbox 和类别 |
| `/perception/target_observation` | RGB-D 解算后的 3D 目标观测 |
| `/grasp/target_pose` | 当前目标中心位姿 |
| `/grasp/trigger` | 抓取触发入口，本阶段不要发布 |
| `/grasp/status` | 抓取状态 |

当前测试中观测到的一个目标输出示例：

```text
frame_id = base_link
class_id = hsv_candidate
confidence = 0.93
position = (0.425, 0.042, 0.616) m
dimensions = (0.019, 0.045, 0.022) m
source = rgbd_mask_geometry
```

该位置是否合理需要用户通过 RViz 实物对照判断，尚未确认。此前也出现过 HSV 框过大、
定位离谱的情况，说明 HSV-only 在复杂画面中容易把高饱和非目标区域选进去。

## 当前用到或准备使用的模型

仓库当前可见权重：

```text
yolov8s-world.pt
yolo11n-seg.pt
sam2_t.pt
```

用途：

- `yolov8s-world.pt`：YOLOWorld 开放词汇检测，用于在 HSV 候选框内确认 `cube`、
  `block`、`small cube`、`toy block`、`artificial object` 等词汇。
- `yolo11n-seg.pt`：Ultralytics YOLO segmentation 预训练权重，可作为自定义实例分割
  训练初值，不应直接期待它识别项目自定义物块。
- `sam2_t.pt`：SAM/SAM2 风格分割权重，目标是在已知候选框内精细抠 mask。它不负责判断
  “是不是目标”，只负责细化边界。

当前运行环境问题：

- 系统 Python 没有 `ultralytics`，所以 `open_vocab_enabled:=true` 会自动退回 HSV-only。
- 如果要启用 YOLOWorld/SAM，需要先处理 Python 环境。已有脚本：

```bash
scripts/setup_target_perception.sh
```

后续建议优先让 ROS launch 明确使用同一个虚拟环境，避免“权重在，但节点 Python 环境找不到
ultralytics”的问题。

## 参考路线

当前设计采用三阶段识别分工：

```text
HSV
  快速找高饱和候选区域，适合黄色/高饱和小方块初期调试

YOLOWorld
  对每个 HSV 候选 crop 做开放词汇语义确认，减少夹爪/线缆/背景误检

SAM/SAM2
  对已确认候选进行像素级分割，提高深度点云裁剪精度
```

搜索阶段和夹取阶段可以使用不同模型：

- 搜索阶段：重召回、低算力、允许粗框，推荐 HSV + YOLOWorld。
- 夹取阶段：重精度、目标通常只有一个，推荐框内 SAM 或自定义实例分割。

对于三类目标：

1. 高饱和方木块：HSV + YOLOWorld 足够做早期验证。
2. 纯色特殊多面体：YOLOWorld 可能辅助，但最终建议自定义实例分割。
3. 岩石样本：与洞穴背景接近，必须采集实车数据训练自定义实例分割，HSV 不适用。

FoundationPose 仅适合有确定 CAD/网格模型的已知物体。形状各异的自然岩石不适合直接用
FoundationPose 统一估计 6D 姿态。

## 已知问题与下一步规划

优先级 1：确认可视化中的几何关系

- 用户观察 RViz 中相机、点云、机械臂、工具点、目标物块是否和实物大致一致。
- 如果目标 marker 明显不在实物位置，先排查 `tool_T_camera` 和 `target_localizer` 的
  TF 使用方向。
- 如果点云整体方向错，优先查 `camera_color_optical_frame` 与
  `gemini335_color_optical_frame` 的关系。

优先级 2：修正正式 TF/URDF

- 当前 `handeye_tf_preview_node.py` 是临时预览，不是正式系统。
- 需要把 `tool_T_camera` 写入正式 launch 或 URDF。
- 同步更新 `grasp_coordinator`、URDF、MoveIt 配置中的机械臂实测臂长：
  `0.07545 / 0.11391 / 0.11000 / 0.15601`。

优先级 3：稳定机械臂控制

- 保持 `motion_solver` 低日志。
- 避免高频 `$ARM:READ` 抢占串口。
- 必要时为机械臂 topic 命令增加发送合并/去抖，避免同一时刻 arm 和 gripper 连发造成
  串口拥塞。
- 如果键盘控制仍抽搐，给 `keyboard_arm_ik_controller.py` 指定单一键盘 event 设备。

优先级 4：目标识别可靠化

- 先让 HSV-only 在桌面小方块上稳定输出合理尺寸和位置。
- 修正 `ignore_rois`，过滤固定入镜的高饱和夹爪区域。
- 安装并统一 `ultralytics` 环境，启用 YOLOWorld 对全部 HSV 候选做确认。
- 再启用 SAM/SAM2，仅在夹取阶段使用。

优先级 5：抓取前闭环验证

- 暂时不要发布 `/grasp/trigger`。
- 先只观察 `/grasp/target_pose` 与 RViz marker。
- 确认位置合理后，让机械臂手动接近目标，检查视觉定位是否随视角变化仍稳定。
- 再做“只规划不执行”或低速单步执行。
- 最后才打开自动抓取。

## 推荐恢复现场命令

相机：

```bash
source install/setup.bash
ros2 launch lunar_rover_sensors gemini335_test.launch.py start_rviz:=false
```

机械臂控制，低日志：

```bash
source install/setup.bash
ros2 run lunar_rover_control motion_solver.py --ros-args \
  --params-file install/lunar_rover_control/share/lunar_rover_control/config/arm_hardware.yaml \
  -p use_sim_time:=false \
  -p h7_port:=/dev/ttyACM0 \
  -p arm_read_period:=0.50 \
  --log-level motion_solver:=warn
```

IK 键盘控制：

```bash
source install/setup.bash
ros2 run lunar_rover_control keyboard_arm_ik_controller.py
```

视觉检测和三维定位，禁用自动抓取：

```bash
source install/setup.bash
ros2 launch lunar_rover_manipulation real_grasping.launch.py \
  detector_mode:=fusion \
  open_vocab_enabled:=true \
  sam_enabled:=false \
  active_search:=false \
  auto_trigger_grasp:=false \
  periodic_sweep:=false
```

临时手眼 TF：

```bash
source install/setup.bash
python3 tools/handeye_tf_preview_node.py
```

临时机械臂和目标 marker：

```bash
source install/setup.bash
python3 tools/arm_grasp_rviz_markers_node.py
```

RViz 3D 观察：

```bash
source install/setup.bash
rviz2 -d generated/calibration/handeye_samples/20260829_152555/arm_target_3d_preview.rviz
```

检查：

```bash
ros2 topic hz /joint_states
ros2 topic echo /perception/target_observation --once
ros2 run tf2_ros tf2_echo base_link gemini335_color_optical_frame -r 1
```
