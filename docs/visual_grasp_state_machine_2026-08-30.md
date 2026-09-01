# 实车视觉抓取状态机设计

更新时间：2026-08-30 17:51 CST

更新：2026-08-31 00:00 CST。状态机已按实车测试反馈调整为“默认中止、手动开始搜索、HSV 锁定后远距位姿统计、再工作区判定”的流程。

本文记录 2026-08-30 实车视觉抓取闭环的下一阶段实施方案。当前已经验证的基础能力包括：

- Gemini 335L RGB-D 相机 ROS 链路。
- HSV 高饱和目标候选、YOLO-World 开放词汇确认、可选 SAM 分割框架。
- RGB-D 目标三维位置解算与 RViz 目标 marker 显示。
- ChArUco 初步手眼标定与 `base_link` 下相机、工具点、机械臂简化模型显示。
- `motion_solver.py` 统一持有 H7 串口，负责机械臂控制、夹爪控制和 `/joint_states` 发布。
- ROS 端机械臂 IK 键盘控制，已同步实测臂长与朝向桌面的 IK 初始位。

后续目标是把这些离散能力组合成自动抓取一轮物块的状态机。

## 总体目标

目标物分为三类：

- 高饱和纯色方木块。
- 高饱和纯色人造不规则多面体。
- 接近真实岩石外观的岩石样本。

当前第一阶段先面向高饱和目标验证完整闭环。YOLO-World 后续会替换为特训过的实例分割或检测模型；状态机接口应保持模型无关。

目标物理包络按不超过 `50 x 50 x 50 mm` 设计。夹取阶段最终闭合夹爪时，当前建议从约 `45 deg` 开度闭合到约 `25 deg`，对应 ROS 夹爪位移从约 `0.0225 m` 降到约 `0.0125 m`。

## 状态机

当前代码状态（2026-08-31）：

```text
ABORT / idle
  -> WIDE_SEARCH / wide_search
  -> TARGET_LOCKED
  -> POSE_AVERAGING / precise_hsv_sam
  -> WORKSPACE_CHECK
  -> APPROACH_WITH_CHASSIS 或 MOVE_TO_GRASP
```

要点：

- Web 端初始显示 `ABORT: 未启动，等待开始搜索`，只有按下“开始搜索”才发布 `/grasp/command=start` 并进入广域搜索。
- `WIDE_SEARCH` 中仍使用 HSV 多候选 + YOLOWorld 开放词汇确认。
- YOLOWorld 确认后，`target_fusion_detector_node.py` 会记录目标 mask 的 HSV 中位值；此后 `locked_hsv` 与 `precise_hsv_sam` 只接受接近该 HSV 指纹的候选，避免绿色框跳到其他高饱和物体。
- `POSE_AVERAGING` 阶段发布 `precise_hsv_sam`，使用锁定 HSV + 低频 SAM 继续观测目标。
- 三维位姿统计只接受相机系深度 `camera_z >= 0.25 m` 且 `camera_z <= 1.20 m` 的样本。低于 25 cm 时不更新目标位姿，用于避开 Gemini 335L 近距离深度不稳定区。
- 默认需要 6 个样本，5 秒滑动窗口内最大空间离散度不超过 5 cm，才生成平均目标位置。
- 平均完成后才进入 `WORKSPACE_CHECK`，并用该平均位置判断是否在机械臂工具点球形可达包络中。
- `MOVE_TO_GRASP` 目前是 TF + 视觉伺服增量移动的入口状态，硬件动作默认尚未自动下发；接线测试前应先确认该状态显示和目标平均位置合理。

新增可视状态：

- `/grasp/pose_average_status`：远距位姿统计状态，Web 端显示采样数量、相机深度、空间离散度和平均结果。
- `/grasp/workspace_status`：工作区判定结果，继续由 Web 端显示。

关键参数位于 `src/lunar_rover_manipulation/config/grasping.yaml`：

```yaml
grasp_state_machine:
  ros__parameters:
    camera_frame: camera_color_optical_frame
    pose_average_required_samples: 6
    pose_average_window_sec: 5.0
    pose_average_min_camera_depth_m: 0.25
    pose_average_max_camera_depth_m: 1.20
    pose_average_max_spread_m: 0.05
    workspace_center_x_m: 0.13
    workspace_center_y_m: 0.0
    workspace_center_z_m: 0.056
    workspace_radius_m: 0.45537
```

```text
IDLE
  -> WIDE_SEARCH
  -> TARGET_LOCKED
  -> APPROACH_WITH_CHASSIS
  -> DEPLOY_GRASP
  -> INCREMENTAL_VISUAL_SERVO
  -> FINAL_ALIGN
  -> CLOSE_GRIPPER
  -> RETREAT_HOME
  -> DONE
```

异常状态：

```text
TARGET_LOST
RECOVERY_SEARCH
ABORT
```

### 1. IDLE

等待抓取任务启动。触发方式可以先保留 `/grasp/trigger` 或键盘 `c`，后续接入导航任务目标时再扩展。

进入条件：

- 用户触发抓取。
- 机械臂控制链路可用。
- 相机 RGB-D 链路可用。
- 当前没有正在执行的抓取任务。

动作：

- 发布 `/grasp/active=true`，让底盘/导航控制链路知道抓取任务开始。
- 清空旧目标缓存。
- 初始化状态机内部目标 ID、HSV 区间、历史三维观测队列。

### 2. WIDE_SEARCH

广泛搜寻模式。该模式用于尽快发现可疑目标，不追求最高精度。

感知链路：

```text
/camera/color/image_raw/compressed
  -> HSV 多候选框
  -> YOLO-World 开放词汇确认
  -> /perception/target_detection
  -> /perception/debug/fusion_overlay/compressed
```

当前实现基础：

- `target_fusion_detector_node.py` 已支持 HSV proposals。
- `open_vocab_enabled=true` 时调用 YOLOWorld。
- 当前现场 CPU 参数建议：
  - `open_vocab_max_candidates=2`
  - `open_vocab_imgsz=256`
  - `sam_enabled=false`

扫描动作：

- J1 小幅慢速左右摆动，扩大水平方向视野。
- J4 小幅慢速俯仰摆动，扩大上下方向视野。
- 摆动幅度应小，避免机械臂遮挡相机过多或影响车体稳定。
- 初始建议：
  - J1：以当前角度为中心，`+-8 deg`。
  - J4：以当前角度为中心，`+-6 deg`。
  - 单周期 `6-10 s`。

锁定条件：

- 某个 HSV 候选通过 YOLOWorld 确认。
- overlay 中该候选文字颜色变红，`stage=open_vocab`。
- 连续 `N` 帧或 `T` 秒内位置相近，避免单帧误检。初始建议：
  - `N=2-3`
  - 图像中心差小于候选框对角线的 `0.5` 倍。

进入下一状态时保存：

- 目标 2D bbox。
- 目标初始 mask。
- YOLO class label 与 confidence。
- 目标 HSV 统计区间。
- 初步 RGB-D 三维位置。

### 3. TARGET_LOCKED

目标锁定模式。此后不再跑 YOLOWorld，降低 CPU 负载和识别延迟，只追踪已确认目标。

动作：

- 停止广泛搜索摆臂，让机械臂回到当前安全姿态或保持静止。
- 将识别器切换到“目标专属 HSV 模式”。
- 专属 HSV 区间由锁定帧 mask 内像素统计得到。

HSV 区间建议：

- 在锁定 mask 内统计 HSV 中位数和分位数。
- H 使用环形色相距离，避免红色跨 0/179 边界。
- 初始范围：
  - H：目标色相 `p10-p90` 再扩展 `+-8`。
  - S：`max(60, p10 - 20)` 到 `255`。
  - V：`max(40, p10 - 20)` 到 `255`。
- 继续应用固定夹爪 ignore ROI。

空间门控：

- 只接受靠近锁定目标历史三维位置的 HSV 连通域。
- 初始 3D gate 半径 `0.12-0.18 m`。
- 图像 gate 可使用上一帧 bbox 外扩 `1.5-2.0` 倍。

输出：

- `/grasp/locked_target_2d`
- `/grasp/locked_target_hsv`
- `/perception/target_observation`

当前代码缺口：

- `target_fusion_detector_node.py` 需要增加运行时模式切换：
  - `mode=wide_search | locked_hsv | precise_hsv_sam`
  - 或拆成独立节点 `locked_hsv_tracker_node.py`。
- 需要新增服务或 topic 传入锁定 HSV 区间。

### 4. APPROACH_WITH_CHASSIS

车体接近模式。目标已经确认，但可能尚未进入机械臂有效工作区。

目标：

- 调整车体 yaw，让目标位于车体正前方。
- 缓慢前进，使目标进入机械臂工作区间。
- 保持导航/雷达链路受控，避免机械臂动作挡住雷达时继续依赖新雷达数据。

控制建议：

- 使用目标在 `base_link` 下的位置 `p = [x, y, z]`。
- yaw 控制目标：减小 `atan2(y, x)`。
- 前进控制目标：让 `x` 落入机械臂舒适工作区。
- 初始工作区建议：
  - `x = 0.18-0.30 m`
  - `|y| < 0.08 m`
  - `z` 在桌面/地面目标合理范围内。

速度建议：

- yaw 角速度或舵轮转向应限幅。
- 前进速度 `0.03-0.08 m/s`。
- 如果目标观测超时超过 `1.0 s`，停止底盘。

进入下一状态条件：

- 目标在机械臂工作区内。
- 目标观测连续稳定：
  - 最近 `3-5` 个三维点标准差小于 `20-30 mm`。
- 车体速度接近 0，并静置 `0.5-1.0 s`。

当前代码缺口：

- 需要新增底盘接近控制节点或在 grasp coordinator 中发布低速 `/cmd_vel`/手动 mux 命令。
- 需要和现有 `nav_to_manual_mux`/键盘劫持逻辑定义优先级：抓取接近控制应高于 Nav2，低于急停/人工键盘。

### 5. DEPLOY_GRASP

部署夹取模式。车体停止，机械臂开始接管任务。

动作：

- 停止车体控制。
- 停止接收新的雷达点云对导航地图/局部 costmap 的影响，保留已有地图和定位。
- 感知模式切换为：

```text
locked HSV 粗定位
  -> SAM/SAM2 mask 精分割
  -> depth 反投影
  -> base_link / arm_base_link 下目标三维位置
```

当前 CPU 上 SAM 可能慢，建议先做两级开关：

- 调试模式：HSV mask + depth。
- 精抓模式：HSV bbox 引导 SAM，每 `1-3 s` 更新一次精 mask。

进入下一状态时保存：

- 部署时刻目标三维位置 `p_target_base_0`。
- 工具中心位置 `p_tool_base_0`。
- 目标在 RGB 图像中的中心 `u_target_0`。
- 工具中心投影到 RGB 图像的位置 `u_tool_0`。

### 6. INCREMENTAL_VISUAL_SERVO

增量视觉伺服模式。机械臂每隔数秒向目标移动一次，不做连续高速闭环，降低误差和控制风险。

每一轮增量包含：

1. 获取当前 `/joint_states`，通过 FK 计算工具中心 `p_tool_base`。
2. 获取当前目标三维观测 `p_target_base`。
3. 使用手眼 TF 将工具中心投影到 RGB 图像，得到 `u_tool`。
4. 使用目标 mask/bbox 得到目标图像中心 `u_target`。
5. 计算两个方向向量：
   - TF 空间向量 `v_tf`
   - 纯视觉伺服向量 `v_img`
6. 按权重融合并归一化。
7. 生成一个小步长 IK 目标并下发。
8. 等机械臂到位并静止，再进入下一轮。

#### TF 空间向量

```text
v_tf = normalize(p_target_base - p_tool_base)
```

如果要避免工具点直接撞击目标，可使用目标前方安全点：

```text
p_goal = p_target_base - approach_axis * standoff
v_tf = normalize(p_goal - p_tool_base)
```

初始 `standoff = 20-40 mm`，最后几轮逐步减小。

#### 纯视觉伺服向量

视觉伺服目标是让图像中工具中心逐步靠近目标中心，同时向目标深度方向靠近。

输入：

- `u_tool = [tool_px, tool_py]`
- `u_target = [target_px, target_py]`
- 当前目标深度 `z_target_camera`
- 当前工具中心由 TF 转到相机坐标 `p_tool_camera`

图像平面方向：

```text
du = u_target - u_tool
```

将像素方向反投影成相机坐标下的近似横向方向：

```text
ray_tool = K^-1 [u_tool.x, u_tool.y, 1]
ray_target = K^-1 [u_target.x, u_target.y, 1]
v_lateral_camera = normalize(ray_target - ray_tool)
```

深度方向：

```text
v_depth_camera = camera_optical_z_axis
```

如果目标比工具更远，则沿相机光轴向前；如果工具已超过目标深度，则减小该分量。

合成：

```text
v_img_camera = normalize(alpha_lateral * v_lateral_camera + alpha_depth * v_depth_camera)
v_img_base = R_base_camera * v_img_camera
```

初始建议：

- `alpha_lateral = clamp(norm(du) / 120 px, 0.2, 1.0)`
- `alpha_depth = 0.6`

#### 权重融合

用户提出的策略：视觉伺服权重随接近过程先升后降，近似正弦曲线。

定义进度：

```text
d0 = 部署模式开始时工具到目标距离
d = 当前工具到目标距离
progress = clamp((d0 - d) / max(d0 - d_final, eps), 0, 1)
```

视觉权重：

```text
w_img = w_peak * sin(pi * progress)
w_tf = 1 - w_img
```

初始建议：

- `w_peak = 0.65`
- `d_final = 20-30 mm`

融合方向：

```text
v = normalize(w_tf * v_tf + w_img * v_img_base)
```

含义：

- 初期：主要相信 TF 三维大方向，避免纯图像误差把机械臂带偏。
- 中期：提高视觉伺服权重，补偿手眼标定和机械臂模型误差，避免丢目标。
- 末期：降低视觉伺服权重，主要使用近距离连续 RGB-D 解算得到的 3D 目标位姿。

#### 增量步长

初始建议：

- 距离 `d > 120 mm`：步长 `25-35 mm`。
- `60 mm < d <= 120 mm`：步长 `12-20 mm`。
- `d <= 60 mm`：步长 `5-10 mm`。

每步应检查：

- IK 是否可解。
- 关节目标是否超过限位。
- 目标是否仍在图像内。
- 深度是否有效。
- 目标 mask 面积是否合理。

如果连续 `2-3` 轮丢目标，进入 `RECOVERY_SEARCH` 或退回 `TARGET_LOCKED`。

### 7. FINAL_ALIGN

最终对准模式。工具中心已经接近目标，但夹爪左右位置需要校准。

用户提出的当前经验判断：

- 视觉判断物块 HSV 区域是否占据画面左下半区。
- 如果物块 HSV 范围超出左下半区，说明夹爪需要往右轻微调整。
- 如果 HSV 在左下半区占比太低，说明夹爪需要往左轻微调整。

建议实现为可调参数，不写死画面区域：

```text
final_roi = [0, image_height * 0.5, image_width * 0.5, image_height * 0.5]
```

计算：

- `target_area_total`：目标 HSV mask 总面积。
- `target_area_in_final_roi`：落在左下 ROI 内的目标面积。
- `ratio = target_area_in_final_roi / target_area_total`。
- `overflow_right`：目标 mask 是否明显越过 ROI 右边界。

控制：

- `overflow_right=true`：工具点向车体右侧小步修正。
- `ratio < min_final_roi_ratio`：工具点向车体左侧小步修正。
- 初始侧向步长 `3-6 mm`。
- 最多修正 `3-5` 次。

进入闭合条件：

- 工具点到目标估计距离小于 `15-25 mm`。
- 目标 mask 仍可见。
- 左下 ROI 占比在阈值范围内。
- 当前夹爪开度约 `45 deg`。

### 8. CLOSE_GRIPPER

下发夹爪闭合。

当前建议：

```text
J5: 45 deg -> 25 deg
```

当前 ROS 映射：

```text
0-90 deg -> 0-0.045 m
25 deg -> 0.0125 m
45 deg -> 0.0225 m
```

topic 控制可通过：

```text
/gripper_controller/commands
```

或 FollowJointTrajectory：

```text
/gripper_controller/follow_joint_trajectory
```

闭合后等待 `0.5-1.0 s`，再进入收回。

### 9. RETREAT_HOME

机械臂回到朝向桌面的 IK 初始位，完成一轮抓取。

当前保存的 IK home：

```text
home_degrees = [96.754752, 201.804458, 183.510522, 110.293936, 45.0]
```

注意：

- 如果夹爪已经夹住目标，回 home 时 J5 不应重新打开到 45 deg。
- 实现上应拆成：
  - `arm_home_degrees = [96.754752, 201.804458, 183.510522, 110.293936]`
  - `carry_gripper_deg = 25.0`

完成后：

- 发布 `/grasp/active=false`。
- 解除底盘/导航锁。
- 状态进入 `DONE` 或回到 `IDLE`。

## 推荐 ROS 接口

### 状态机节点

建议新增或重构为：

```text
grasp_state_machine_node.py
```

职责：

- 管理上述状态。
- 订阅目标检测、目标三维观测、相机内参、关节状态。
- 发布感知模式切换命令。
- 发布底盘低速接近命令。
- 发布机械臂增量 IK 目标。
- 发布夹爪开合命令。

### 感知模式切换

建议新增 topic：

```text
/perception/grasp_mode std_msgs/String
```

取值：

```text
wide_search
locked_hsv
precise_hsv_sam
idle
```

或者用 service：

```text
/perception/set_grasp_mode
```

短期调试建议用 topic，便于命令行切换。

### 锁定目标颜色

建议新增消息或参数服务，短期可用 `Float32MultiArray`：

```text
/perception/locked_hsv_range
```

内容：

```text
[h_min, h_max, s_min, s_max, v_min, v_max]
```

如果 H 跨 0 边界，应支持两段 H 区间。长期建议定义专用 msg：

```text
HsvTargetRange.msg
```

### 底盘接近控制

短期可以接现有手动/导航 mux，新增抓取接近控制源：

```text
/grasp/chassis_cmd
```

优先级建议：

```text
急停/人工键盘 > 抓取接近控制 > Nav2 > 空闲
```

抓取部署后底盘必须停止。

### 机械臂增量控制

当前 `motion_solver.py` 已支持：

```text
/arm_controller/joint_trajectory
/gripper_controller/commands
/gripper_controller/follow_joint_trajectory
/joint_states
```

状态机可以复用 `keyboard_arm_ik_controller.py` 中的 FK/IK 算法，后续建议把 IK 几何函数抽到共享模块，避免键盘控制器和自动抓取状态机各自维护一份。

## 实现顺序

### 阶段 A：只做状态机骨架

- 新增 `grasp_state_machine_node.py`。
- 支持 `IDLE -> WIDE_SEARCH -> TARGET_LOCKED`。
- 暂不控制底盘和机械臂，只发布状态和日志。
- 验证 YOLOWorld 红字候选能触发锁定。

### 阶段 B：目标专属 HSV 跟踪

- 给 `target_fusion_detector_node.py` 增加 `locked_hsv` 模式。
- 从锁定 mask 自动生成 HSV 区间。
- 停止 YOLOWorld，只用 HSV + 图像/空间 gate 跟踪同一目标。
- 验证 overlay 能稳定只框住目标，而不是夹爪或背景。

### 阶段 C：车体接近

- 增加低速底盘控制输出。
- 目标超时立即停。
- 先在架空或安全低速环境验证 yaw 对正，再验证前进进入工作区。

### 阶段 D：部署夹取感知

- 增加 `precise_hsv_sam` 模式。
- SAM 默认低频运行，HSV 高频辅助。
- `target_localizer` 使用精 mask 输出更可靠的目标中心和尺寸。

### 阶段 E：机械臂增量视觉伺服

- 复用 IK 控制器的 FK/IK。
- 实现 `v_tf`、`v_img`、正弦权重融合。
- 每步移动后等待稳定并重新观测。
- RViz 中显示：
  - `p_target_base`
  - `p_tool_base`
  - `v_tf`
  - `v_img`
  - `v_fused`

### 阶段 F：最终对准与夹爪闭合

- 实现左下 ROI 判断。
- 小步侧向微调。
- J5 从 `45 deg` 闭合到 `25 deg`。
- 回到 IK home，保持夹爪闭合。

## 风险与注意事项

- CPU 上 YOLOWorld 已验证约 `3-4 Hz`，SAM 会更慢。广泛搜索可接受，部署夹取阶段应低频调用 SAM。
- 手眼标定当前仍是初步结果，增量闭环必须依赖每步重新观测，不能一次性规划到底。
- 机械臂会遮挡相机和雷达。部署夹取后应停止依赖新的雷达点云更新。
- `motion_solver.py` 当前是 H7 串口唯一 owner，不应再同时启动 `arm_joint_state_reader_node.py`。
- 当前 `motion_solver.py` 已修复嵌入 `$STD` 后的 `$ARM:RESP` 解析问题，`/joint_states` 可由 motion solver 直接发布。
- 目标接近时 mask 可能占满画面或只剩局部可见，因此视觉伺服权重末期应下降，避免被局部 mask 形状误导。
- 增量移动必须保守，任何一步 IK 不可解、目标丢失、深度无效、关节反馈异常都应停在当前位置等待人工确认。

## 当前需要新增或重构的代码

- `grasp_state_machine_node.py`：主状态机。
- `locked_hsv_tracker_node.py` 或扩展 `target_fusion_detector_node.py`：目标专属 HSV 跟踪。
- `visual_servo_utils.py`：工具点投影、图像伺服向量、TF 向量融合。
- `arm_kinematics.py`：从 `keyboard_arm_ik_controller.py` 抽出的 FK/IK 与限位处理。
- RViz marker 扩展：显示工具点、目标点、融合方向向量和状态机文本。

短期最小可行路径：

```text
先不动底盘
  -> YOLOWorld 红字锁定
  -> 自动切 locked HSV
  -> RViz 显示锁定目标和工具点
  -> 机械臂只做单步 5 mm 增量试探
  -> 人工确认后再开放多步闭环
```
