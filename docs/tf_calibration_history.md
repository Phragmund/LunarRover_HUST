# TF 标定版本与回退维护记录

最后更新：2026-10-06。历史配置版本：`20260930_all12_park`（相机支架换装前，仅作历史回退记录）。新支架三组融合第一版 `20261006_mount1_fused_v1_park` 已登记并切换为 active；新支架 campaign 见 [`tf_calibration_campaign_20261006.md`](tf_calibration_campaign_20261006.md)。

## 1. 权威来源与范围

本文记录目前能从代码、保存的标定报告及在线 ROS 图验证的 TF；不从对话中的历史试调值猜测参数，也不使用已废弃的 tmp PDF。历史结果不等同于每个版本都上线过或通过实机验收。

- [版本注册表](../src/lunar_rover_manipulation/config/handeye_calibrations.json)：全精度变换、8 个具名标定、选用版本、FK 参数；实际发布节点直接读取这一份文件。
- [更新前在线快照](calibration/tf_snapshots/20260930_before_handeye_update.json)：全部收到的 15 条静态边和 6 条动态边，含时刻、关节值及发布者。
- [URDF 全关节原点快照](calibration/tf_model_origins_20260930.json)：Rover_0 的 29 个关节、Rover_1 参考模型的 24 个关节；包含原点 XYZ、RPY、四元数、4×4 矩阵、轴和关节类型。
- [本次 12 组数值样本](calibration/handeye_samples/20260930_131448.jsonl)与[新旧对比报告](calibration/20260930_handeye_comparison.json)：已移入可版本管理的 docs，不依赖被忽略的 generated 目录。
- [更新后在线快照](calibration/tf_snapshots/20260930_after_handeye_update.json)：部署核验通过，11 条静态边、6 条动态边；在线矩阵与注册表一致、TF 单父节点且无环、base→camera 查找成功、5 秒内收到 18 帧融合画面。
- [新支架第一版在线快照](calibration/tf_snapshots/20261006_mount1_fused_v1_online.json)：`20261006_mount1_fused_v1_park` 已由注册表和 `/handeye/active_calibration` 发布，`tf2_echo` 与矩阵一致；快照时相机未输出 RGB 帧，属于相机流可用性问题，不改变外参部署结果。
- 发布入口：[手眼节点](../tools/handeye_tf_preview_node.py)；版本选择：[管理工具](../tools/handeye_calibration_registry.py)；受控重启：[重启脚本](../scripts/restart_handeye_tf.sh)。

导航运行产生的位姿、关节运动后的 TF 不是永久外参。动态值只作为有时间戳的快照归档，回退时恢复计算方式和配置，不能把快照当静态 TF 重发。

## 2. 单位和方向约定

`parent_T_child` 表示 child 坐标系在 parent 中的位姿：`p_parent = R * p_child + t`。XYZ 单位 m，四元数顺序 `[x,y,z,w]`；表中 RPY 单位度、顺序 roll/pitch/yaw，矩阵采用 `Rz(yaw) @ Ry(pitch) @ Rx(roll)`。URDF 快照的原始 RPY 单位是 rad，字段名已区分。

手眼外参的父坐标系是 `handeye_tool_frame`，其 XYZ 不能直接解释成车体左、右、上、下。旋转差使用相对旋转的轴角大小，不能用三个欧拉角差代替。

## 3. 实机视觉抓取 TF 链及归属

```text
base_link → arm_base_link → handeye_tool_frame → camera_color_optical_frame
                                               ├→ gemini335_color_optical_frame
                                               └→ camera_depth_optical_frame
                                                   └→ gemini335_depth_optical_frame
base_link → charuco_board_frame  （仅标定板显示）
```

| 变换 | 类型/来源 | 当前值或规则 |
|---|---|---|
| base_link → arm_base_link | 静态，注册表；RSP 同值发布 | t=[0.13, 0, 0.056]；q=[0, 0, 0, 1] |
| arm_base_link → handeye_tool_frame | 动态，手眼节点 | /joint_states 的 J1～J4 + 第 5 节 FK，20 Hz |
| handeye_tool_frame → camera_color_optical_frame | 静态，手眼节点唯一负责 | 当前版本 `20261006_mount1_fused_v1_park`，见下表 |
| camera_color_optical_frame → gemini335_color_optical_frame | 静态别名，手眼节点 | t=[0,0,0]，q=[0,0,0,1] |
| camera_color_optical_frame → camera_depth_optical_frame | 静态别名，手眼节点 | t=[0,0,0]，q=[0,0,0,1] |
| camera_depth_optical_frame → gemini335_depth_optical_frame | 静态别名，手眼节点 | t=[0,0,0]，q=[0,0,0,1] |
| base_link → charuco_board_frame | 静态，仅 RViz 显示 | 新版本 t=[0.740441940, 0.048214465, 0.204237932]；q=[-0.364687141, 0.611388906, 0.565821521, -0.415996275] |

深度恒等别名沿用当前 RGB 对齐的深度链路约定，不是重新测量了相机内部 RGB/深度外参。切换为未配准的原始深度时必须重新检查该约定。

2026-09-30 发现并处理的归属冲突：

- 旧 RSP 发布 `gemini335_color_optical_frame → camera_color_optical_frame`，手眼节点同时发布反向别名。
- 旧 RSP 从粗略 URDF 安装值发布相机光学帧，手眼节点也从标定值发布相同 child，存在多父节点/覆盖。
- 实机 RSP 必须以 `use_sim:=false external_camera_tf:=true` 生成 Rover_0 URDF；相机配置维持 `publish_tf: false`。该开关只禁止 RSP 发布四个相机光学帧，保留相机外壳可视模型和所有雷达/机械臂模型关节，不把标定写死进 URDF。
- `base_link → arm_base_link` 的两份发布目前同值；若以后修改机械臂安装位置，注册表与模型必须一起核对，不能产生两份不同值。
- 历史相机光学边保存在快照中用于审计，不应恢复其双向别名发布。回退手眼参数不回退这个归属修正。

## 4. 手眼标定版本总表

以下均为 `handeye_tool_frame → camera_color_optical_frame`。表格便于比较，注册表保留全精度矩阵/四元数作为恢复依据。

| 版本 ID | 样本数 / 方法 | XYZ (m) | RPY (deg) | 用途 |
|---|---|---|---|---|
| `20260829_all27_tsai` | 27 / TSAI | -0.033266606, -0.070193203, 0.048909094 | 101.411041, 2.493820, 83.507236 | 历史/备选求解结果，未宣称实机验收 |
| `20260829_filtered21_tsai` | 21 / TSAI | -0.036126818, -0.075786008, 0.040861100 | 102.952070, 2.458380, 83.671021 | 历史/备选求解结果，未宣称实机验收 |
| `20260829_robust17_park` | 17 / PARK | -0.043339740, -0.042208126, 0.025279712 | 102.645455, -0.752233, 88.311304 | 历史/备选求解结果，未宣称实机验收 |
| `20260831_filtered36_park` | 36 / PARK | -0.035597434, -0.042355152, 0.034338323 | 98.606286, 1.728528, 87.846924 | 更新前在线值，首选回退点 |
| `20260930_all12_park` | 12 / PARK | -0.034622807, -0.014470450, 0.024134876 | 90.842446, 7.153228, 93.109314 | 当前默认 |
| `20260930_filtered9_park` | 9 / PARK | -0.032782785, -0.012516691, 0.025684489 | 91.049693, 8.075155, 92.745485 | 历史/备选求解结果，未宣称实机验收 |
| `20261006_mount1_fused_v1_park` | 244 / PARK+SO(3) 等权组融合 | -0.041300931, -0.033105358, 0.023544997 | 96.744951, 2.171249, 88.135807 | 新支架第一版，已上线；待实物抓取验收 |

当前新支架第一版完整值：

```text
translation_xyz_m = [-0.041300930793331, -0.033105358015466, 0.023544996820221]
quaternion_xyzw   = [0.528225626749242, 0.528829341248085, 0.451758700488351, 0.487063950229908]
rpy_deg_xyz       = [96.74495136942197, 2.171248767858557, 88.13580708965193]
```

换装前旧版本 `20260930_all12_park` 的完整值（仅历史回退）：

```text
translation_xyz_m = [-0.03462280662651714, -0.014470450185839449, 0.024134876054882966]
quaternion_xyzw   = [0.45706823349582104, 0.5462443356266495, 0.4780445664177037, 0.5139836069791717]
rpy_deg_xyz       = [90.84244596468261, 7.153227616506778, 93.10931428266143]
```

新 12 组版本对旧在线版本：平移差 29.708868 mm，相对旋转差 11.118402°。按 ≥35 个角点、图像/关节时差 ≤100 ms 筛选后采用第 1～9 组的备选，与旧版差 31.195270 mm / 11.295550°。12 组版与 9 组版彼此相差 3.099057 mm / 1.022375°。

同一批 12 组样本中，标定板平移平均残差由旧版 17.797313 mm 降至新版 11.774476 mm，最大残差由 35.335631 mm 降至 19.319832 mm。采样期间底盘和板固定是求解前提；残差属于样本内自洽性，不代表已验证的绝对抓取精度。

本次采用全部 12 组，与前次“新 TF”对比口径一致；9 组结果作为可选择备选，没有隐式替换。8 月 31 日报告的 filtered 与 filtered_plus_residual_trim 数值相同，注册表合并为同一回退版本。8 月 29 日保留三份不同结果；不把它们全部标记为曾上线。

旧报告将 FK 输出标记为 `base_T_board_mean_translation_m`，但求解器实际输出在 arm_base_link 中，未包含机械臂安装偏移。为了精确恢复旧显示，旧版本保留原显示位移；新版本显示板位姿时加入 [0.13,0,0.056]。板显示帧不是物块定位输入，不参与直接抓取。

## 5. 动态手眼 FK：回退必须一起检查的几何

| 项目 | 值 |
|---|---|
| ROS→舵机零点 (J1…J5) | [90,180,135,135,0] deg |
| ROS 方向 (J1…J5) | [1,-1,1,1,1] |
| J1–J2 轴间径向偏置 | 0.07545 m |
| 上臂 / 前臂 / 工具 | 0.11391 / 0.11000 / 0.15601 m |
| 动态发布频率 | 20 Hz |
| 关节输入 | /joint_states，按 arm_joint1…arm_joint5 名称排序 |

对 J1～J4：`servo_deg[i] = zero_deg[i] + direction[i] * degrees(ros_position[i])`。
J5 为夹爪开度，`degrees = opening_m * 90 / 0.045`，不参与末端旋转/平移。

使用角度转换为弧度后：

```text
yaw = J1 - 90°
a2 = J2
a3 = J2 + (135° - J3)
a4 = J2 + (135° - J3) + (135° - J4)
r = 0.07545 + 0.11391*sin(a2) + 0.11000*sin(a3) + 0.15601*sin(a4)
z = -0.11391*cos(a2) - 0.11000*cos(a3) - 0.15601*cos(a4)
t_arm_tool = [r*cos(yaw), r*sin(yaw), z]
x_axis = [cos(yaw)*sin(a4), sin(yaw)*sin(a4), -cos(a4)]
y_axis = [-sin(yaw), cos(yaw), 0]
z_axis = normalize(cross(x_axis,y_axis))
R_arm_tool = [x_axis, y_axis, z_axis] （列向量）
```

0.07545 m 是径向偏置，不是竖直抬高。RSP 的 CAD/可视模型链与这个实际 IK/FK 工具链不是可互换的坐标系；不能用 URDF 的 arm_link5 代替 handeye_tool_frame。发布器 FK 与求解器 FK 已逐样本对比一致。

## 6. 其余静态 TF 和模型安装值

以下雷达安装值由 RSP 保持，未随本次手眼更新修改：

| 父 → 子 | XYZ (m) | RPY (deg) |
|---|---|---|
| base_link → sensor_mast_link | [0.06,0,0.05] | [0,0,0] |
| sensor_mast_link → mid360_link | [0,0,0.155] | [0,45,0] |
| mid360_link → livox_frame | [0,0,0] | [0,0,0] |
| mid360_link → livox_imu_frame | [0,0,0] | [0,0,0] |

Rover_0 粗略相机外壳 `arm_link4 → gemini335_link` 为 [0.04,0,0.065] m、[0,-90,0] deg；外壳 → 旧光学帧为 [0.015,0,0] m、[-90,0,-90] deg。这些原值归档，不是实机手眼真值。

Rover_1 参考模型（未在本次实机 ROS 图作为 RSP 输入）另有：
`base_link → cad_base_link`=[0.2745,0.135,0] m、yaw≈-90°；
`cad_base_link → gemini335_link`=[0.135,0.0175,-0.075] m、yaw≈90°；
外壳 → 光学帧=[0.0175,0,0] m、[-90,0,-90] deg。
全部底盘摇臂、车轮、机械臂关节原点见模型 JSON，不仅是本文摘要。

FAST-LIO 内部配置 `fast_lio2_mid360_real.yaml` 的 `mapping.extrinsic_T=[0,0,0]`、`extrinsic_R=I`、`extrinsic_est_en=false`；这是 LIO 内部外参配置，不是额外发布的一条 ROS TF，本次未修改。

## 7. 导航动态 TF 的记录边界

归档时未收到 map/odom 导航边，不能声称它们当时在线，也不能给里程计位姿编造固定值。项目存在不同导航入口，启动时只能选一套：

- `lio_nav_tf_bridge.py`：map→odom=恒等（定时动态重发）；实际代码将校正 LIO 位姿发布为 map→base_link。
- `lio_odometry_tf_bridge.py`：map→odom=恒等静态；LIO 位姿发布为 odom→base_link。
- `map_odom_tf_bridge.py` + 局部里程计：map→odom 来自 LIO 与局部里程计的校正，odom→base_link 来自里程计。

三者不是同一条实际发布方式，不同时启动/回放。导航位置是随时间变化的估计值，回退需对应启动入口、参数与数据，而不是固定六自由度值。本次不调整导航算法。

## 8. 更新前在线完整数值快照

采集时刻 UTC：2026-09-30T05:36:10.757132+00:00。四元数顺序 xyzw；这里如实记录旧图，包括随后移除的相机发布冲突。

| 静态父 → 子 | XYZ (m) | quaternion xyzw |
|---|---|---|
| `base_link → arm_base_link` | 0.130000000, 0.000000000, 0.056000000 | 0.000000000, 0.000000000, 0.000000000, 1.000000000 |
| `base_link → charuco_board_frame` | 0.568216245, 0.171722351, 0.229307515 | -0.477805969, 0.541297839, 0.494822253, -0.483579407 |
| `handeye_tool_frame → camera_color_optical_frame` | -0.035597434, -0.042355152, 0.034338323 | 0.539200069, 0.532964493, 0.444041310, 0.477534763 |
| `camera_color_optical_frame → gemini335_color_optical_frame` | 0.000000000, 0.000000000, 0.000000000 | 0.000000000, 0.000000000, 0.000000000, 1.000000000 |
| `camera_color_optical_frame → camera_depth_optical_frame` | 0.000000000, 0.000000000, 0.000000000 | 0.000000000, 0.000000000, 0.000000000, 1.000000000 |
| `camera_depth_optical_frame → gemini335_depth_optical_frame` | 0.000000000, 0.000000000, 0.000000000 | 0.000000000, 0.000000000, 0.000000000, 1.000000000 |
| `gemini335_color_optical_frame → camera_color_optical_frame` | 0.000000000, 0.000000000, 0.000000000 | 0.000000000, 0.000000000, 0.000000000, 1.000000000 |
| `gemini335_depth_optical_frame → camera_depth_optical_frame` | 0.000000000, 0.000000000, 0.000000000 | 0.000000000, 0.000000000, 0.000000000, 1.000000000 |
| `gemini335_link → gemini335_color_optical_frame` | 0.015000000, 0.000000000, 0.000000000 | 0.500000000, -0.500000000, 0.500000000, -0.500000000 |
| `gemini335_color_optical_frame → gemini335_depth_optical_frame` | 0.000000000, 0.000000000, 0.000000000 | 0.000000000, 0.000000000, 0.000000000, 1.000000000 |
| `arm_link4 → gemini335_link` | 0.040000000, 0.000000000, 0.065000000 | 0.000000000, -0.707106781, 0.000000000, 0.707106781 |
| `mid360_link → livox_frame` | 0.000000000, 0.000000000, 0.000000000 | 0.000000000, 0.000000000, 0.000000000, 1.000000000 |
| `mid360_link → livox_imu_frame` | 0.000000000, 0.000000000, 0.000000000 | 0.000000000, 0.000000000, 0.000000000, 1.000000000 |
| `sensor_mast_link → mid360_link` | 0.000000000, 0.000000000, 0.155000000 | 0.000000000, 0.382683432, 0.000000000, 0.923879533 |
| `base_link → sensor_mast_link` | 0.060000000, 0.000000000, 0.050000000 | 0.000000000, 0.000000000, 0.000000000, 1.000000000 |

| 动态父 → 子（仅该时刻） | XYZ (m) | quaternion xyzw |
|---|---|---|
| `arm_base_link → handeye_tool_frame` | 0.235534825, -0.041298057, 0.220616824 | -0.008699112, -0.099983995, -0.086239960, 0.991206434 |
| `arm_base_link → arm_link1` | 0.000000000, 0.000000000, 0.000000000 | 0.000000000, 0.000000000, -0.086677594, 0.996236415 |
| `arm_link1 → arm_link2` | 0.000000000, 0.000000000, 0.075450000 | 0.000000000, -0.243361092, 0.000000000, 0.969935760 |
| `arm_link2 → arm_link3` | 0.000000000, 0.000000000, 0.113910000 | 0.000000000, 0.530844026, 0.000000000, 0.847469539 |
| `arm_link3 → arm_link4` | 0.000000000, 0.000000000, 0.110000000 | 0.000000000, 0.362641362, 0.000000000, 0.931928775 |
| `arm_link4 → arm_link5` | 0.000000000, 0.020992500, 0.156010000 | 0.000000000, 0.000000000, 0.000000000, 1.000000000 |

动态关节值和原始 stamp 见快照 JSON。不能用这一表作为静态参数加载。

## 9. 抓取补偿与 TF 分开维护

新支架第一轮实机验证采用“纯 TF 直达”基线，以下经验补偿已全部归零；它们不是手眼外参：

| 参数 | 当前配置 |
|---|---|
| tf_direct_j1_offset_deg | 0° |
| tf_direct_j4_image_up_offset_deg | 0° |
| tf_direct_overshoot_m | 0 m |
| tf_direct_vertical_drop_m | 0 m |
| ik_j2_compensation_deg | 0° |
| grasp_z_offset_m / wall_grasp_offset_m | 0 / 0 m |

配置来源 `src/lunar_rover_manipulation/config/grasping.yaml`。J1 完成到位反馈后，J2/J3/J4 直接按融合 TF 目标重新求解并发送，不再叠加固定角度、前伸或竖直下降补偿。

## 10. 查看、切换、回退

首先中止/完成正在执行的抓取，让机械臂与底盘静止。以下命令不发机械臂或底盘运动命令。

```bash
cd /home/phragmund/LunarRover
python3 tools/handeye_calibration_registry.py list

# 持久回退至本次更新前真正在线的版本：
python3 tools/handeye_calibration_registry.py activate 20260831_filtered36_park
bash scripts/restart_handeye_tf.sh

# 恢复此次 12 组新版本：
python3 tools/handeye_calibration_registry.py activate 20260930_all12_park
bash scripts/restart_handeye_tf.sh

# 当前新支架第一版：
python3 tools/handeye_calibration_registry.py activate 20261006_mount1_fused_v1_park
bash scripts/restart_handeye_tf.sh

# 查看节点实际选用版本，而不是只看磁盘默认：
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /handeye/active_calibration --once --qos-durability transient_local
ros2 run tf2_ros tf2_echo handeye_tool_frame camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

管理工具先验证所有变换，再原子保存 active_calibration；非法版本/不一致矩阵拒绝写入。受控重启脚本验证配置、按仓库路径识别并只停止手眼节点，使用共享锁防止与 watchdog 并发启动。旧节点未退出则拒绝再开一个。摄像机、机械臂串口和任务执行器不会被这条命令重启。

完整链路按钮与守护进程统一调用相同入口并读取同一默认版本；重启后不会回到旧硬编码值。直接运行节点可以用 `--ros-args -p calibration_id:=版本ID` 临时指定，但该覆盖不改变持久默认，长期使用推荐 activate。

如果手工重启 RSP，必须指定 `external_camera_tf:=true`；不要为了回退外参重新启用旧相机光学边。发生重复父节点时先修发布归属，再核对矩阵。已有 RViz/TF 缓存若仍保留已停用的旧边，重开该只读显示/TF 客户端后查看；不要反复重启相机。

## 11. 后续维护和验收规则

1. 每次新采样单独存档：时间、板 7×10/方格25 mm/标记18 mm/DICT_5X5_1000、图像内参、关节映射、采样 indices、图像/关节时间差、求解方法、散差、样本 SHA256。
2. 追加具名 profile，不覆盖旧 profile 的数值。说明是否仅候选、是否上线和是否实机验证。
3. 更新前后各留一次 /tf_static、/tf、发布者及关节快照；确认相机各 child 只有一个父节点。
4. 外参验证合格后才修改 active_calibration；只重启相关 TF 发布器，检查长链与相机图像仍正常。
5. 新版本上线之后的抓取精度/目标静止性单独记录。不要把矩阵求解完成等同于硬件抓取验收通过。
6. 回退后核验在线主题与矩阵，并核对补偿配置；经验补偿与 TF 版本分别记录，防止二者相互掩盖。

回归检查：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover -s tools/tests -p test_handeye_calibration.py -v
```

检查覆盖：六份版本合法性、12/9 组重算一致性、旧在线矩阵与板显示精确回退、求解器/发布器 FK 一致、持久切换不篡改旧版本、非法数据拒绝、实机 URDF 光学帧归属。
