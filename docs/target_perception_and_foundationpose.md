# 实车目标搜索、墙面抓取与 FoundationPose 接入

更新时间：2026-08-24

## 当前链路

```text
Gemini 335 RGB
  -> target_detector (HSV 调试后备 / 自定义 YOLO 实例分割)
  -> /perception/target_mask + /perception/target_detection
Gemini 335 registered depth + mask
  -> target_localizer (目标点云、支撑面拟合、墙面/地面分类)
  -> /perception/target_observation
  -> active_search (多帧确认、地图内去重、可选机械臂离散扫描)
C 键 / 可选自动触发
  -> grasp_coordinator
  -> 导航取消、底盘与雷达锁定、预抓取、粗伺服、近端精伺服、闭环末段抓取
```

视觉检测在常规行驶时持续运行，但下面两个会导致机械动作的参数默认关闭：

- `AUTO_TRIGGER_GRASP=false`
- `PERIODIC_ARM_SWEEP=false`

未经实车标定不要打开周期摆臂。按 `C` 触发仍是默认抓取入口。

## 三类目标与检测策略

统一类别名为 `wood_block`、`artificial_object`、`rock_sample`，目标物理包络不超过
`50 x 50 x 50 mm`。高饱和纯色方木块和人造多面体可以先用 HSV 后备链路联调；
岩石样本与洞穴背景近似，必须用真实相机数据训练自定义实例分割模型。通用 COCO 权重
只能作为迁移学习初值，不能直接可靠识别这三类目标。

数据集采用 Ultralytics segmentation 格式，配置位于
`src/lunar_rover_manipulation/config/target_dataset.yaml`。建议至少覆盖：不同距离与像素尺寸、
墙面/地面、遮挡、机械臂入镜、强弱光、运动模糊、真实岩壁难负样本。验证集必须按采集
场景划分，不能把同一段视频的相邻帧随机拆进训练集和验证集。

## 三阶段候选融合链路

实车 NUC 默认使用轻量稳定模式：

```text
HSV 多连通域候选
  -> /perception/debug/fusion_overlay/compressed
  -> /perception/target_mask + /perception/target_detection
```

当安装了可选模型依赖后，可以逐级打开：

```text
HSV 多候选框
  -> YOLO-World / 开放词汇检测确认 colored cube、small block 等提示词
  -> SAM/SAM2 在确认框内精细分割
  -> 深度图定位
```

这条链路的实现节点是 `target_fusion_detector_node.py`。HSV 阶段负责快速找高饱和候选；
开放词汇阶段只做语义确认，不直接输出最终 mask；SAM 阶段只做像素级抠图，不负责判断
类别。任一可选模型加载失败时，节点会自动退回 HSV 结果，避免实车启动失败。

只启用 HSV 多候选融合节点：

```bash
ros2 launch lunar_rover_manipulation real_grasping.launch.py detector_mode:=fusion
```

启用开放词汇确认和 SAM 精分割前，先安装 CPU 推理环境：

```bash
scripts/setup_target_perception.sh
source .deps/target_perception_venv/bin/activate
```

然后在该环境中启动 ROS，或确保 launch 进程使用该 Python 环境：

```bash
ros2 launch lunar_rover_manipulation real_grasping.launch.py \
  detector_mode:=fusion \
  open_vocab_enabled:=true \
  sam_enabled:=true
```

NUC 上建议先只打开 `open_vocab_enabled:=true`，确认帧率还能接受后再打开 `sam_enabled`。
如果 SAM 在 CPU 上太慢，保留 HSV mask 进入深度定位，不阻塞主流程。

```bash
scripts/setup_target_perception.sh
source .deps/target_perception_venv/bin/activate
scripts/train_target_segmentation.py --data /absolute/path/target_dataset.yaml \
  --export-onnx
```

部署训练权重：

```bash
source .deps/target_perception_venv/bin/activate
TARGET_DETECTOR_BACKEND=ultralytics \
TARGET_SEGMENTATION_MODEL=/absolute/path/best.pt \
scripts/run_real_lidar_mapping.sh
```

## 墙面判别与分阶段伺服

`target_localizer` 将 mask 内注册深度反投影为目标点云，并在 mask 外围环带拟合支撑平面。
支撑面法向在 `base_link` 中满足 `|normal.z| <= 0.35` 时判为墙面，满足
`|normal.z| >= 0.70` 时判为地面。大于 `90 mm` 的目标点云包络会被拒绝，防止把墙体
本身当成目标。

抓取分为 `pregrasp -> 粗校正 -> near -> 精校正 -> grasp -> retreat`。粗、精校正使用
新一批 RGB-D 观测重新求目标中心，默认容差分别为 `18 mm` 和 `8 mm`，每阶段最多两次
修正。最后几厘米不继续依赖视觉，因为眼在手上相机会被夹爪和目标遮挡。

当前机械臂只有 4 DoF，墙面法向必须大致落在底座到目标的径向平面内；默认偏差超过
`35 deg` 会拒绝抓取，避免生成机械结构无法实现的姿态。

## FoundationPose 框架

FoundationPose 只用于有确定 CAD/扫描网格的已知实例 6D 位姿估计。每种不规则多面体或
岩石如果形状不同，都需要各自的米制网格；它不能仅凭 `rock_sample` 类别为任意岩石
生成可信 6D 姿态。对象登记模板位于
`src/lunar_rover_manipulation/config/foundationpose_objects.yaml`。

Humble 主链已经提供标准边界：

- 输入：`/foundationpose/image`、`/foundationpose/depth_image`、
  `/foundationpose/camera_info`、`/foundationpose/segmentation`
- 输出：外部 FoundationPose 将 `Detection3DArray` 重映射到
  `/foundationpose/output`
- 桥接结果：`/perception/target_pose_precise`

启用桥节点：

```bash
RUN_FOUNDATIONPOSE_BRIDGE=true scripts/run_real_lidar_mapping.sh
```

官方 Isaac ROS 最新预构建链路使用 ROS 2 Jazzy，原始 NVLabs 项目也要求 CUDA、编译扩展
和单独下载权重。当前电脑没有 Docker/Podman 和 `nvcc`，因此只完成了 Humble 侧接口与
源代码获取脚本，没有假装完成不可运行的 GPU 部署。可先执行
`scripts/fetch_foundationpose.sh` 获取官方源码；后续在兼容容器中启动后，通过相同
`ROS_DOMAIN_ID` 和上述话题连接。

## 第三方许可证

本仓库没有提交 Ultralytics、FoundationPose 源码或权重，只提供可选安装/获取脚本和 ROS
接口。Ultralytics 当前采用 AGPL-3.0 与商业授权双轨；FoundationPose 使用 NVIDIA Source
Code License。对外发布产品或服务前应单独核对所选版本、权重和部署方式的许可证义务。

官方参考：

- https://github.com/ultralytics/ultralytics/blob/main/docs/en/tasks/segment.md
- https://github.com/NVlabs/FoundationPose
- https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html
