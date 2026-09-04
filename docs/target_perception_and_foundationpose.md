# 实车目标搜索、墙面抓取与 FoundationPose 接入

更新时间：2026-09-04

## 当前链路

```text
Gemini 335 RGB
  -> HSV 多候选提议
  -> 自训练 Ultralytics `weights/best.pt` 对同一区域确认
  -> /perception/target_mask + /perception/target_detection
Gemini 335 registered depth + mask
  -> target_localizer (目标点云、支撑面拟合、墙面/地面分类)
  -> /perception/target_observation
  -> active_search (多帧确认、地图内去重、可选机械臂离散扫描)
任务调度 / C 键触发
  -> grasp_state_machine + mission_executive
  -> 暂停探索、底盘与雷达锁定、0.1m 视觉步进、抓稳确认、返航投放
```

视觉检测在常规行驶时持续运行，但下面两个会导致机械动作的参数默认关闭：

- `AUTO_TRIGGER_GRASP=false`
- `PERIODIC_ARM_SWEEP=false`

未经实车标定不要打开周期摆臂。按 `C` 触发仍是默认抓取入口。

## 默认部署目标与检测策略

默认自训练权重 `weights/best.pt` 的部署类别为 `wood_block` 与
`artificial_object`，目标物理包络不超过 `50 x 50 x 50 mm`。默认广域搜索先产生 HSV
候选，再要求 `best.pt` 检测框与候选框的 IoU 不低于 `0.10`，才视为自动确认；HSV 本身
不可替代 YOLO 自动确认。`rock_sample` 保留为数据集
模板类别，尚未包含在该部署权重中。通用 COCO 或 YOLO-World 权重只能作为迁移学习/实验
工具，不能替代自训练模型作为正式确认器。

数据集采用 Ultralytics detect 格式（需要实例分割时才使用 polygon 标签），配置位于
`src/lunar_rover_manipulation/config/target_dataset.yaml`。建议至少覆盖：不同距离与像素尺寸、
墙面/地面、遮挡、机械臂入镜、强弱光、运动模糊、真实岩壁难负样本。验证集必须按采集
场景划分，不能把同一段视频的相邻帧随机拆进训练集和验证集。

## 默认链路与可选融合实验

实车 NUC 默认使用 HSV 提议与自训练模型确认的融合链路：

```text
HSV 多候选
  -> weights/best.pt（Ultralytics detect，候选框空间一致性确认）
  -> /perception/debug/fusion_overlay/compressed
  -> /perception/target_mask + /perception/target_detection
```

YOLO-World 是可选的开放词汇实验确认器，默认关闭。SAM/SAM2 则在目标已经由
`best.pt` 确认并锁定后，才按低频率细化 HSV mask：

```text
HSV 多候选框
  -> best.pt 确认 wood_block / artificial_object
  -> 锁定 HSV 目标
  -> SAM/SAM2 在锁定候选内精细分割（可选）
  -> 深度图定位
```

这条链路的实现节点是 `target_fusion_detector_node.py`。HSV 阶段负责快速找高饱和候选；
`best.pt` 只做语义确认，不直接替换 HSV mask；SAM 阶段只做像素级抠图，不负责判断类别。
若 `best.pt` 加载或推理失败，节点只会保留 `hsv_candidate`，状态机不会自动锁定；操作者
可在 `WIDE_SEARCH` 状态使用“跳过 YOLO 确认”进行显式联调确认。

默认启动即为正式融合链路：

```bash
ros2 launch lunar_rover_manipulation real_grasping.launch.py \
  detector_mode:=fusion \
  segmentation_model:=/absolute/path/to/weights/best.pt
```

启用可选 YOLO-World 实验确认或 SAM 精分割前，先安装推理环境：

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

默认部署权重（`weights/best.pt` 已按启动脚本自动传入）：

```bash
source .deps/target_perception_venv/bin/activate
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

本仓库随初赛源码提交自训练部署权重 `weights/best.pt`，但不提交 Ultralytics 或
FoundationPose 源码，也不提交 FoundationPose 权重。Ultralytics 当前采用 AGPL-3.0 与商业
授权双轨；FoundationPose 使用 NVIDIA Source Code License。对外发布产品或服务前应单独
核对所选版本、权重和部署方式的许可证义务。

官方参考：

- https://github.com/ultralytics/ultralytics/blob/main/docs/en/tasks/segment.md
- https://github.com/NVlabs/FoundationPose
- https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html
