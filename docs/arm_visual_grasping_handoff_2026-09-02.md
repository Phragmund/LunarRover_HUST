# 实车机械臂视觉抓取总交接

日期：2026-09-02 CST  
代码基线：`50f2276 feat: add real rover visual grasp pipeline`

本文是下一轮对话或接手开发者恢复实车联调的主入口。旧文档 `real_arm_visual_grasping_handoff.md` 和 `visual_grasp_state_machine_2026-08-30.md` 保留为历史记录，其中部分“未实现”状态已经过期；以本文和当前代码为准。

## 1. 当前范围

当前验证目标是高饱和小型物块（不超过 `50 x 50 x 50 mm`）：Gemini 335L 检测、YOLOWorld 确认、HSV/SAM 锁定、RGB-D 三维定位、机械臂自动接近和夹爪动作。

当前尚未完成：存放动作、FoundationPose 实机闭环、正式 URDF 手眼外参固化。底盘接近采用受限的视觉步进控制；首次联调仍必须预留人工中止空间。

## 2. 实车几何和坐标

不要用 `Rover_0` 老仿真模型校正实车抓取。实测机械臂几何：

| 参数 | 数值 |
| --- | --- |
| J1-J2 轴最短偏移 | `75.45 mm` |
| J2-J3 | `113.91 mm` |
| J3-J4 | `110.00 mm` |
| J4-tool | `156.01 mm` |

- `J1=0°` 朝车体右，`J1=90°` 朝车体前，正角继续转向左。
- `J2=180°` 指上，`J2=90°` 指前。
- `J3/J4=135°` 时各自与前一 link 共线；增大 J3/J4 会向车前倾。
- `J5=45°` 是当前预备开度，`J5=27°` 是试验性夹紧开度。
- ROS 基座：`+X` 车前、`+Y` 车左、`+Z` 向上。

人工确认的朝桌面 IK 初始位：

```text
[96.754752, 201.804458, 183.510522, 110.293936, 45.0] deg
```

## 3. 硬件、TF 和标定

- 相机：Orbbec Gemini 335L，`640x400@15Hz`，正常应枚举为 USB 3.2。
- H7 串口常用稳定路径：`/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_357635623333-if00`。每次拔插后重新检查，不能硬编码 `/dev/ttyACM0`。
- 手眼临时 TF 由 `tools/handeye_tf_preview_node.py` 发布，服务于 RViz、RGB-D 目标解算和图像投影。它尚不是可宣称高精度的最终外参。
- ChArUco：A4 `7x10`，square `25 mm`，marker `18 mm`，`DICT_5X5_1000`。生成文件与样本在被 Git 忽略的 `generated/`。

历史工具像素参考是 `(454,0)`，用于紫色工具点/早期视觉伺服。当前预采样居中使用的是画面中心 `(320,200)`；两者用途不同，不可混淆。

## 4. 控制链路和已知限制

`motion_solver.py` 是唯一应占用 H7 串口的节点。

| 用途 | 话题 |
| --- | --- |
| J1-J4 | `/arm_controller/joint_trajectory` |
| J5 | `/gripper_controller/commands` |
| H7 原始发送/回传 | `/h7/raw_command` / `/h7/rx_frame` |
| 关节状态 | `/joint_states` |

J5 topic 的第一项是夹爪 ROS 位移；第二项若存在是动作时长秒数，旧单值调用仍兼容。

### 不能忽略的坑

1. `/joint_states` 不是可靠的物理到位反馈。`motion_solver._send_arm_positions()` 在 `$ARM:MULTI` 发出后立即更新内部关节位置，因此会产生“命令影子”。此前用它判断 J1 到位，约 50 ms 就错误释放了其余关节。
2. 当前顺序动作依靠 `time_from_start + 余量`，不是真实闭环到位。要根治必须让 H7 明确区分命令目标和 `$ARM:RESP` 真回读。
3. H7 偶发漏解析 `$ARM:MULTI`，J4 较常见；关键纯 TF 指令当前补发 3 次、间隔 `0.12 s`。
4. 过高 `$ARM:READ` 频率和 INFO 日志会导致控制卡顿。实车使用 `arm_read_period:=0.50`、`--log-level motion_solver:=warn`。
5. 曾观察到突发大幅抽动和 `$ARM:ERR,TIMEOUT!`/截断回传，尚未完成根因分析。不要把它误归因为 IK。

## 5. 感知链路

```text
/camera/color/image_raw/compressed
  -> HSV proposals -> YOLOWorld confirmation
  -> locked HSV + low-rate SAM
  -> target_localizer (RGB-D + TF)
  -> /perception/target_observation (base_link)
```

- HSV：高频候选和锁定后追踪。YOLO 确认后锁住 mask 的 HSV profile，后续不得跳到其他高饱和物体。
- YOLOWorld：只负责广域确认；overlay 红字代表可锁定的开放词汇确认。
- SAM：锁定后低频细化 mask，主要服务夹取/深度区域。
- 当前实测启动：`open_vocab_enabled=true`、`sam_enabled=true`、`sam_async_enabled=false`。

感知坑点：

- RViz raw 图像显示慢不代表相机低帧；实际应测 compressed RGB，正常约 15 Hz。融合 overlay 因 CPU YOLO/SAM 正常约 3.7 Hz。
- Gemini 335L 近距离深度不稳定。远距平均只接受 `0.20m <= camera_z <= 1.20m`。
- SAM 曾选中大片地面背景。`crop_expand_ratio` 已从 `0.35` 收紧为 `0.10`，同时使用 HSV seed coverage/IoU/紧凑度门控；不要随意调回 0.35。
- 夹爪在画面中的高饱和固定区域必须由 `ignore_rois` 屏蔽。
- `.venv_yolo`、模型权重和 `weights/` 不提交 Git；按 `dependencies/target_perception.txt` 与 `scripts/setup_target_perception.sh` 恢复。

## 6. 当前自动状态机

核心文件：

```text
src/lunar_rover_manipulation/scripts/grasp_state_machine_node.py
src/lunar_rover_manipulation/config/grasping.yaml
```

默认状态必须为：

```text
ABORT: 未启动，等待开始搜索
```

网页“开始搜索”发送 `/grasp/command=start_search`。有效流程：

```text
ABORT -> WIDE_SEARCH -> TARGET_LOCKED -> POSE_AVERAGING
      -> AUTO_CENTERING -> four-corner sampling
      -> WORKSPACE_CHECK -> MOVE_TO_GRASP
      -> staged TF reach -> J5/J2 post sequence
```

### 6.1 YOLO 锁定后的预采样视觉居中

锁定后切至 `precise_hsv_sam`，先用 J1/J4 gimbal 映射把锁定 HSV 目标拉向 RGB 中心 `(320,200)`。

- 现行增益：J1/J4 均 `0.05 deg/px`。
- 死区：`12 px`。
- 单次运动：`0.5 s`，额外等待 `0.15 s`，多次小步收敛。
- 上限：12 步或 15 秒。失败时停止自动采样并停在 `POSE_AVERAGING`，要求用户中止后重试，不允许盲目前进。
- 复用 `gimbal_j1_sign`、`gimbal_j4_sign`。`gimbal_servo_enabled=false` 不会禁用该预采样居中。

若视觉方向反了，先中止并检查 sign，不要靠改 TF 补偿掩盖问题。

### 6.2 四角远距采样

目标居中后，把当时姿态作为四角中心，J2/J3/J5 保持不变：

```text
(J1-8, J4-8)
(J1-8, J4+8)
(J1+8, J4+8)
(J1+8, J4-8)
```

每角点动作 `1.0 s`、再等 `0.15 s`，静置后采 10 帧有效深度，四批均值形成 `base_link` 目标。若深度范围、TF 或锁定视觉无效，则不伪造样本。工作区外进入 `APPROACH_WITH_CHASSIS`：以 `0.04 m/s` 前进，每次限 `0.10 m`，停车 `1 s` 后使用新的 RGB-D 观测重新判定；目标偏离 RGB 中心超过 `20 px` 时叠加限幅 `0.12 rad/s` 的阿克曼小转向。视觉丢失即发零速度。该通路通过 `/grasp/chassis_cmd_vel -> nav_to_manual_mux -> /manual_control` 下发，非零手机杆量仍优先。

通过工作区后状态机自动调用一次 `execute_increment_step()`；不会重复触发。

### 6.3 当前纯 TF 试验抓取序列

1. J5 先用 1 秒回 `45°`，补发 3 次。
2. J1 单独到 TF IK 目标加左补偿 `+4°`，动作 2 秒，等待动作窗口。
3. J1 完成后以新工具点计算方向 `u`，末端最终目标：

```text
target_final = tf_target + 0.035 * normalize(tf_target - tool_after_j1)
```

4. J1 保持第一段角度，重解 J2/J3/J4；J4 叠加画面向上 `+7°`，相机倒装下等效原始 J4 `-7°`。
5. TF 末段后等 1 秒，J5 用 1 秒到 `27°` 并补发；再等 1 秒，J2 用 1 秒回 `180°` 并补发。

每次有效单步会向下列 Git 忽略文件追加一条 JSONL，用于统计目标分布：

```text
generated/logs/grasp_increment_samples.jsonl
```

以上数值是当前现场经验补偿，不是几何真值。手眼误差、舵机误差或新相机安装位置变化后必须重新验证。

## 7. RViz 和网页控制台

RViz 配置：

```text
generated/calibration/handeye_samples/20260829_152555/arm_target_3d_preview.rviz
```

辅助节点：`tools/handeye_tf_preview_node.py`、`tools/arm_grasp_rviz_markers_node.py`。MarkerArray 正确话题是 `/visualization/arm_grasp_markers`，不是 RViz 默认的 `/visualization_marker_array`。

网页控制台：

```text
scripts/restart_ros_web_console.sh
http://127.0.0.1:8099
ws://127.0.0.1:8099/ros
```

页面用 `file://` 打开时曾回退到旧端口 `8088`，已改为 8099；仍推荐通过 HTTP 地址打开。若进程在但网页无法向 ROS 发命令，检查：

```bash
ros2 node list | rg 'ros_web_console|grasp_state_machine'
```

控制台不在 ROS 图中时重启 `scripts/restart_ros_web_console.sh`，不能只刷新浏览器。

## 8. 恢复启动顺序

先查设备：

```bash
ls -l /dev/serial/by-id/
lsusb | rg -i 'Orbbec|Gemini'
```

相机：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lunar_rover_sensors gemini335_test.launch.py start_rviz:=false
```

H7/机械臂，禁用底盘：

```bash
python3 src/lunar_rover_control/scripts/motion_solver.py --ros-args \
  --params-file src/lunar_rover_control/config/arm_hardware.yaml \
  -p h7_port:=/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_357635623333-if00 \
  -p raw_command_topic:=/h7/raw_command -p raw_feedback_topic:=/h7/rx_frame \
  -p enable_drive_on_start:=false -p use_sim_time:=false \
  -p arm_read_period:=0.50 --log-level motion_solver:=warn
```

TF/RViz：

```bash
python3 tools/handeye_tf_preview_node.py
python3 tools/arm_grasp_rviz_markers_node.py
rviz2 -d generated/calibration/handeye_samples/20260829_152555/arm_target_3d_preview.rviz
```

感知：

```bash
python3 src/lunar_rover_manipulation/scripts/target_localizer_node.py --ros-args --params-file src/lunar_rover_manipulation/config/grasping.yaml
/home/phragmund/LunarRover/.venv_yolo/bin/python src/lunar_rover_manipulation/scripts/target_fusion_detector_node.py --ros-args --params-file src/lunar_rover_manipulation/config/grasping.yaml -p open_vocab_enabled:=true -p sam_enabled:=true -p sam_async_enabled:=false
```

状态机试验参数：

```bash
python3 src/lunar_rover_manipulation/scripts/grasp_state_machine_node.py --ros-args --params-file src/lunar_rover_manipulation/config/grasping.yaml -p visual_servo_enabled:=false -p gimbal_servo_enabled:=false -p tf_direct_target_enabled:=true -p tf_direct_target_duration_sec:=2.0 -p tf_direct_overshoot_m:=0.035 -p tf_direct_j1_offset_deg:=4.0 -p tf_direct_j4_image_up_offset_deg:=7.0 -p tf_direct_pre_gripper_reset_enabled:=true -p tf_direct_post_sequence_enabled:=true
```

最后：

```bash
scripts/restart_ros_web_console.sh
```

健康基线：compressed RGB 约 15 Hz，fusion overlay 约 3.7 Hz，初始 `/grasp/status` 为 `ABORT`。

## 9. 推荐联调顺序和未解决风险

1. 先观察相机、网页、RViz、关节读数，不放物块。
2. 放高饱和测试块，只开始搜索，确认 HSV 框、YOLO 红字、锁定 HSV 正确。
3. 检查 `AUTO_CENTERING` 的方向和小步收敛，再允许四角动作。
4. 检查四个 `AUTO_MOVE pose=1/4...4/4`，确认 J2/J3/J5 未被采样动作改变。
5. 确认平均目标、工作区和黄点合理后，才允许自动单步动作。
6. 首轮完整动作使用无价值测试块，随时可中止，重点看 J4 漏命令和 H7 超时。

当前抓稳确认在 J5 闭合后使用锁定 HSV：目标需连续 3 帧占据 RGB 左下 ROI 至少 1%，否则不回收 J2 并报告 `GRASP_UNCONFIRMED`。关节角回读作为当前阶段的到位反馈。后续优先级为：手眼再标定/误差评估、抓稳/失败统计、存放动作与雷达遮挡门控。

## 10. Git 说明

本地 `.venv_yolo/`、`weights/`、`*.pt`、`generated/`、厂商 IDE 导出和网页打包文件已忽略。恢复仓库后需要自行准备推理环境、权重、标定样本和现场日志；不要把这些多 GB 本地资产直接提交到普通 GitHub 仓库。
