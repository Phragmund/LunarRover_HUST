# 实车纯雷达 Nav2 导航·建图·探索全链路交接文档

> 日期：2026-08-30
> 适用：六轮摇臂月球车实车（本机 NUC 上位机）
> 面向：后续接手的 agent / 维护人员

---

## 1. 一句话总结

实车已跑通「**纯雷达（Livox Mid-360）建图 + FAST-LIO2 定位 + 2.5D 地形建图 + Nav2 导航 + 前沿探索**」全链路，底盘由 **H7 下位机** 驱动；支持**手机网页遥控**与**「电脑接管」（Nav2 接管底盘）**两种模式，并带手机优先仲裁。

---

## 2. 链路架构

### 2.1 数据流

```text
Livox Mid-360 (/livox/lidar 10Hz + /livox/imu 200Hz)
  → FAST-LIO2 (fastlio_mapping, 实车配置 lidar_type=4)
  → /localization/lio_odometry_raw + cloud_registered_raw (camera_init 帧)
  → fast_lio2_map_frame_node (水平校正 + 帧转换)
  → /localization/lio_odometry (map→base_link) + cloud_registered (map 帧)
  → lio_nav_tf_bridge (发布 map→odom 恒等 + map→base_link TF)
  → terrain_mapper → /terrain/traversability (2.5D 可通行代价图)
  → Nav2 (8 个生命周期节点, TerrainLayer) → /cmd_vel_nav
  → twist_stamper → /navigation/chassis_cmd_vel (TwistStamped)
  → nav_to_manual_mux (手机优先仲裁 + TwistStamped→ManualControl 阿克曼转换)
  → /manual_control → motion_solver → H7 USB CDC (/dev/ttyACM0)
```

### 2.2 控制两条路径（关键）

| 路径 | 消息 | 末端 |
|---|---|---|
| 手机手动 | web_joystick_server → `/web/manual_control`(ManualControl) → mux | motion_solver → H7 |
| Nav2 电脑接管 | Nav2 → `/cmd_vel_nav`(Twist) → twist_stamper → `/navigation/chassis_cmd_vel`(TwistStamped) → mux | motion_solver → H7 |

- **motion_solver 独占 H7 串口**，是唯一底盘驱动节点；两路最终都汇到 `/manual_control`。
- 旧链路 `chassis_driver_node`(USB2CANFD) / `chassis_cmd_mux` **已不用于实车手动控制**，仅保留给仿真/Nav 历史用途。

### 2.3 话题清单（实车关键）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/livox/lidar` | PointCloud2 | 雷达原始点云 10Hz |
| `/livox/imu` | Imu | 雷达内置 IMU 200Hz |
| `/localization/lio_odometry` | Odometry | 校正后定位 map→base_link |
| `/localization/cloud_registered` | PointCloud2 | 校正后注册点云（map 帧，0.5Hz） |
| `/terrain/traversability` | OccupancyGrid | 2.5D 可通行代价图 |
| `/cmd_vel_nav` | Twist | Nav2 原始速度输出（velocity_smoother 前） |
| `/navigation/chassis_cmd_vel` | TwistStamped | twist_stamper 输出（给 mux） |
| `/web/manual_control` | ManualControl | 手机遥控输出（给 mux） |
| `/manual_control` | ManualControl | mux 输出（给 motion_solver） |

---

## 3. 硬件与前置条件

- **Livox Mid-360**：IP `192.168.2.204`，本机网口 `enp7s0 = 192.168.2.50/24`（实测二手改过子网的单元，非出厂默认 `192.168.1.1XX`）。
- **H7 下位机**：USB CDC 虚拟串口 `/dev/ttyACM0`（STM32 VCP `0483:5740`，udev 规则已配 0666）。
- 相机（Gemini 335L）**本链路不融合**（未标定），需融合时再单独接入。
- 已构建工作区（`install/` 有 `fast_lio`、`livox_ros_driver2`、各 `lunar_rover_*` 包）。

---

## 4. 一键启动

```bash
cd /home/phragmund/LunarRover
bash scripts/restart_real_nav2.sh
```

脚本 5 步（已含门控等待）：
1. 杀掉所有旧进程。
2. `ros2 daemon` 重启 + 重新生成 URDF（`/tmp/lunar_rover_real.urdf`）与 RSP 参数。
3. 启动 雷达 / robot_state_publisher / FAST-LIO2 / map_frame×2 / lio_nav_tf_bridge / nav_to_manual_mux。
4. **等待 `/localization/lio_odometry` 首帧**后，启动 terrain_mapper / Nav2 / twist_stamper / web_joystick / motion_solver。
5. **等待 Nav2 `/compute_path_to_pose` action**后，启动 frontier_explorer / RViz。

启动完成后关键检查：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 lifecycle get /controller_server   # 期望 active
ros2 lifecycle get /planner_server       # 期望 active
ros2 lifecycle get /bt_navigator         # 期望 active
tail /tmp/frontier.log                   # 期望 Selected goal ...
```

日志都在 `/tmp/*.log`（lidar / fastlio / mapframe_main / mapframe_viz / terrain / nav2 / twist / web / motion / frontier / rviz）。

---

## 5. 关键文件清单（本次新增/修改）

| 文件 | 作用 |
|---|---|
| `scripts/restart_real_nav2.sh` | 一键干净重启脚本 |
| `src/lunar_rover_navigation/scripts/nav_to_manual_mux.py` | Nav2→ManualControl 桥接 + 手机优先仲裁 |
| `src/lunar_rover_navigation/scripts/lio_nav_tf_bridge.py` | map→odom(恒等) + map→base_link TF 定时发布 |
| `src/lunar_rover_navigation/config/nav2_fast_lio2_params_real.yaml` | Nav2 实车参数（use_sim_time=false, 速度） |
| `src/lunar_rover_navigation/config/twist_stamper_real.yaml` | twist_stamper 实车参数（input=/cmd_vel_nav） |
| `src/lunar_rover_mapping/config/terrain_mapper_real.yaml` | terrain 实车参数（去天花板滤波） |
| `src/lunar_rover_exploration/config/frontier_explorer_real.yaml` | frontier 实车参数（use_sim_time=false） |
| `src/lunar_rover_mapping/rviz/real_nav2.rviz` | RViz 视图（点云/代价图/路径） |
| `src/lunar_rover_msgs/msg/ManualControl.msg` | 新增 `auto_takeover` 字段 |
| `src/lunar_rover_description/Rover_0/urdf/lunar_rover.urdf.xacro` | 雷达安装角 35°→45° |

---

## 6. 关键参数

### 6.1 速度（手机 vs Nav2）

- 手机满杆：`motion_solver` 的 `max_speed=1.5 r/s`，线速度 ≈ `1.5×0.0625 ≈ 0.094 m/s`。
- Nav2（电脑接管）速度 = 手机满杆的 **2/3**：
  - `vx_max=0.06 m/s`、`wz_max=0.30 rad/s`（`nav2_fast_lio2_params_real.yaml`）。
  - twist_stamper 钳位 `max_linear_x=0.07`、`max_angular_z=0.35`。
- `nav_to_manual_mux` 的 `v_max=0.09375` 是「手机满杆基准」，用于把 Nav2 的 m/s 换算成手机油门杆量，**不是限速**，不要改。

### 6.2 坐标系 / 重力对齐（重要，本次核心修复）

- FAST-LIO2 的 `camera_init` 帧**对齐 IMU 初始姿态，不是重力**。
- map_frame 节点用 `imu_mount_pitch_source:=startup_imu_gravity`，启动时从 `/livox/imu` 线加速度**实测 IMU 实际俯仰/滚转**（而非写死 URDF 值），把 map 帧对齐到**重力铅锤**。
- 实测例：`pitch=+40.602 deg`（URDF 写 45°，差值 ~4.4° 即车架实际俯仰）。
- URDF 中雷达机械安装角保持 **45°**（相对 base_link），但只是参考/yaw，水平对齐以 IMU 实测为准。
- `level_correction_source:=startup` 保留做小残余校正。

---

## 7. 手机遥控 + 电脑接管

- 手机入口：`http://<本机WiFi IP>:8000/?token=rover2026`（IP 用 `hostname -I` 查，会变，见 9.6）。
- 页面按钮：摇杆（上下=前进后退，左右=阿克曼转向）、⟲⟳ 原地转、⛔急停、✅解除急停、🔌断开转矩(R)、🔄重启设零(T)、🛠标定四轮(C)、**💻电脑接管**。

**「电脑接管」按钮（默认关闭）**：

| 状态 | 行为 |
|---|---|
| 关闭（默认） | 手机摇杆控制底盘，松开即停（不自动切 Nav2） |
| 开启 | Nav2 接管底盘；手机一推摇杆即临时手动接管，松开交还 Nav2 |
| 急停 ⛔ | 强制停止并自动关闭「电脑接管」 |

---

## 8. 停止链路

```bash
pkill -TERM -f 'fastlio_mapping' ; pkill -TERM -f 'livox_ros_driver2'
pkill -TERM -f 'fast_lio2_map_frame' ; pkill -TERM -f 'nav_to_manual_mux'
pkill -TERM -f 'terrain_mapper' ; pkill -TERM -f 'twist_stamper'
pkill -TERM -f 'web_joystick_server' ; pkill -TERM -f 'motion_solver'
pkill -TERM -f 'frontier_explorer' ; pkill -TERM -f 'navigation.launch'
pkill -TERM -x controller_server ; pkill -TERM -x planner_server
pkill -TERM -x bt_navigator ; pkill -TERM -x behavior_server
pkill -TERM -x smoother_server ; pkill -TERM -x velocity_smoother
pkill -TERM -x lifecycle_manager ; pkill -TERM -x rviz2
```

---

## 9. 常见问题与解决方案（踩坑记录）

### 9.1 点云整体倾斜（重力不对齐）★本次核心
- **现象**：RViz 点云朝车体正向（pitch）倾斜约 10~15°。
- **根因**：`imu_mount_pitch_source=urdf` 只补机械安装角（35°/45°），没补车架实际俯仰；FAST-LIO 的 `camera_init` 又不重力对齐。
- **解决**：改 `imu_mount_pitch_source:=startup_imu_gravity`（实测 IMU 重力）。日志会打印 `pitch=+40.602 deg` 这类实测值。
- **验证**：`grep -E 'Calibrated effective IMU|pitch=' /tmp/mapframe_main.log`。

### 9.2 雷达掉线后 FAST-LIO 发散
- **现象**：掉线重连后，`map_frame` 疯狂打印 `Dropping invalid FAST-LIO2 pose: position=(-1xxxx, ...)`（里程计漂到几千米）。
- **根因**：雷达断流期间 FAST-LIO 的 IMU 积分漂移，重连后状态已坏。
- **解决**：重启 FAST-LIO2 + map_frame（一键脚本搞定）。**掉线优先查物理接线/上电**（`ping 192.168.2.204`、`ip -br addr show enp7s0` 应为 UP）。

### 9.3 Nav2 生命周期没全部激活
- **现象**：`ros2 lifecycle get /planner_server` 显示 `unconfigured`，只有 controller_server active；frontier 报 `PathQueryHelper: /compute_path_to_pose not available` 崩溃。
- **根因**：Nav2 的 lifecycle bringup 在 TF 未就绪时启动，部分节点激活失败；frontier 抢跑。
- **解决**：
  1. 保证 TF 链先就绪（脚本已门控：先等 `/localization/lio_odometry` 再起 Nav2）。
  2. 手动兜底：`ros2 lifecycle set /planner_server configure` 再 `activate`（bt_navigator / behavior_server / costmap 同理）。
  3. frontier 必须在 Nav2 `/compute_path_to_pose` 出现后再启动（脚本已门控）。

### 9.4 cmd_vel 主题对不上（Nav2 不驱动）
- **现象**：Nav2 发布 `/cmd_vel_nav`，但 twist_stamper 订阅 `/cmd_vel`（0 publisher）。
- **根因**：nav2_bringup 默认把 controller_server 的 `cmd_vel` 重映射为 `cmd_vel_nav`。
- **解决**：`twist_stamper_real.yaml` 的 `input_topic` 用 `/cmd_vel_nav`。

### 9.5 标定四轮按钮失效
- **根因**：`nav_to_manual_mux` 的 `manual_active` 条件漏了 `msg.calibrate`，把标定消息当「空闲」丢弃。
- **解决**：`manual_active` 条件需包含 `or msg.calibrate`（relax/rearm 同理）。

### 9.6 手机连不上
- **根因**：WiFi IP 会变（如 `10.19.144.127` → `192.168.43.113` 热点网段），不是服务挂（服务绑定 `0.0.0.0:8000`）。
- **解决**：`hostname -I` 查当前 IP，用 `http://<当前IP>:8000/?token=rover2026`。

### 9.7 一键脚本等待循环卡住/超时
- **根因**：杀掉旧进程后 ROS 2 daemon 状态残留，`ros2 topic echo`/`ros2 action list` 失败。
- **解决**：脚本 `[2/5]` 已加 `ros2 daemon stop` + `ros2 daemon start`。

### 9.8 帧命名 body vs livox_frame
- FAST-LIO2 源码写死里程计 `camera_init → body`；实车驱动把 IMU 数据帧标成 `livox_frame`。map_frame 节点用 `lio_body_frame:=livox_frame` 启动即可对齐（两者同为 IMU 帧）。

### 9.9 motion_solver 启动报 /dev/ttyACM0 不存在
- H7 重插会短暂掉设备/换号。先 `ls /dev/ttyACM*` 确认，再启动；若换号需改 `h7_port`。

---

## 10. 待办 / 未完成项

- 雷达↔相机外参标定（当前纯雷达，未融合 Gemini）。
- `Optimizer fail to compute path`（MPPI 局部控制器）：局部代价图把车附近标成障碍时会出现，需观察 RViz local costmap 是否过保守，再调 MPPI 的 `inflation_radius/collision_margin_distance` 或 terrain 代价层。
- terrain_mapper 的 registered cloud 输入仅 0.5Hz（map_frame 可视化节点限速），导航实时性可再优化（提高 `registered_cloud_max_rate_hz`）。
