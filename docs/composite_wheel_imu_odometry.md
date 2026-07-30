# 组合轮式 + IMU 里程计构建与标定说明

本文档记录当前底盘里程计的设计、标定方法，以及它在 SLAM 建图中的接入方式。当前路线不新增传感器，只使用已有轮组关节反馈、IMU 和仿真真值进行标定验证。

## 目标

原始轮式里程计在迷宫手操中主要问题是航向漂移。轮组反馈对前向距离较可靠，但在原地转向、贴墙、转向过渡和轮胎滑移时，用轮速反推 yaw 会明显失真。

当前组合里程计的设计目标是：

- 轮组关节反馈只提供前向速度/距离；
- IMU 提供 yaw 和 yaw rate；
- 不使用 IMU 加速度双积分计算位置；
- 保留原始轮式里程计作为对比和诊断；
- 让 SLAM 默认使用组合里程计发布的 `odom -> base_link`。

## 输入与输出

组合里程计节点：

```text
node:  /composite_wheel_imu_odometry
file:  src/lunar_rover_mapping/scripts/composite_wheel_imu_odometry_node.py
config: src/lunar_rover_mapping/config/composite_odometry.yaml
```

输入：

```text
/joint_states   6 个轮速 + 4 个转向角
/livox/imu      IMU orientation + angular velocity
/tf             base_link <- livox_imu_frame 固定外参
```

输出：

```text
/localization/composite_odometry
TF: odom -> base_link
```

原始轮式里程计仍发布：

```text
/localization/odometry
```

但在 mapping 启动链路中不再发布 `odom -> base_link` TF，避免和组合里程计冲突。

## 算法结构

### 轮组前向速度估计

节点直接订阅 `/joint_states`，从 6 个轮子的角速度和 4 个转向关节角计算底盘平面速度。

每个轮子建立一条约束：

```text
r * wheel_angular_velocity =
  cos(steering_angle) * vx
  + (-wheel_y * cos(steering_angle) + wheel_x * sin(steering_angle)) * wz
```

其中：

- `r` 是轮半径；
- `vx` 是底盘前向速度；
- `wz` 是由轮组反推的角速度；
- 前/后转向轮使用实际转向角；
- 中间固定轮转向角为 0。

节点对所有有效轮子做最小二乘拟合。当前组合里程计只使用拟合出的 `vx` 作为距离源，不使用轮组反推的 `wz` 做航向积分。

### IMU 航向估计

IMU 安装存在固定 pitch 外参。节点不会硬编码安装角，而是通过 TF 查询：

```text
base_link <- livox_imu_frame
```

当前仿真中识别到的外参为：

```text
roll = 0 deg
pitch = 35 deg
yaw = 0 deg
```

节点启动后使用前若干 IMU 样本完成：

- 初始 yaw 零点定义；
- gyro z bias 估计；
- 后续 yaw 由 IMU orientation 修正；
- yaw rate 由 IMU angular velocity 经外参旋转后给出。

### 平面位姿积分

最终平面位姿积分使用：

```text
x += vx * cos(yaw_mid) * dt
y += vx * sin(yaw_mid) * dt
yaw = imu_yaw
```

也就是说：

- 距离来自轮组；
- 方向来自 IMU；
- 不对 IMU linear acceleration 做位置积分。

## 抗异常处理

当前实现包含以下保护：

- `max_wheel_angular_velocity`：轮速限幅；
- `wheel_outlier_residual_mps`：剔除残差过大的轮子；
- `min_wheels_for_fit`：有效轮数不足时不更新轮速；
- `wheel_velocity_timeout`：关节反馈过期则速度归零；
- `stationary_wheel_speed_threshold_mps`：低速静止阈值；
- `stationary_yaw_rate_threshold_radps`：低 yaw rate 静止阈值；
- `max_forward_accel_mps2`：前向速度变化率限制；
- `max_dt`：仿真/节点卡顿时跳过过大的积分步长。

这些保护主要用于减少长按键、贴墙、轮子局部异常和仿真步长抖动造成的轨迹污染。

## 标定参数

配置文件：

```text
src/lunar_rover_mapping/config/composite_odometry.yaml
```

关键参数：

```yaml
wheel_radius: 0.0625
wheelbase: 0.39
track_width: 0.345
forward_velocity_scale: 1.0
```

当前轮径来自真实轮组直径：

```text
diameter = 125 mm
radius   = 62.5 mm = 0.0625 m
```

`forward_velocity_scale` 是前向距离标定旋钮。平地标定结果建议值约为 `1.0006`，因此当前保持 `1.0`。

## 标定方法

推荐先在平地标定，不要直接在迷宫里调参数。迷宫中的贴墙、碰撞、卡角会把物理接触误差混入里程计参数。

标定工具：

```text
tools/run_odometry_calibration.py
```

典型流程：

```bash
cd /home/phragmund/LunarRover
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lunar_rover_mapping sim_mapping.launch.py world:=flat gui:=false mapping_rviz:=false
```

另一个终端运行：

```bash
cd /home/phragmund/LunarRover
source /opt/ros/humble/setup.bash
source install/setup.bash
tools/run_odometry_calibration.py
```

标定动作序列：

```text
静止
直行
停止
原地左转
停止
再次直行
最终停止
```

当前一次平地标定结果：

```text
truth path_length: 2.980 m

wheel:
  final_xy = 0.216 m
  rms_xy   = 0.086 m
  final_yaw = -10.37 deg
  rms_yaw   = 6.22 deg

composite:
  final_xy = 0.030 m
  rms_xy   = 0.073 m
  final_yaw = -1.45 deg
  rms_yaw   = 6.32 deg

suggested forward_velocity_scale ~= 1.0006
```

解释：

- 组合里程计最终 XY 误差明显小于原始轮式里程计；
- 组合里程计最终航向误差明显小于原始轮式里程计；
- 路径长度比例接近 1，轮径/前向比例暂不需要调整。

## RViz 可视化

mapping RViz 中默认显示：

```text
黄色 Gazebo Truth Path
蓝色 Wheel Odometry Path
绿色 Composite Wheel+IMU Path
```

判断原则：

- 平地短距离：绿色应基本贴合黄色；
- 原地转向后继续直行：绿色航向应明显优于蓝色；
- 长时间迷宫手操：绿色位置误差可累计，但应显著小于蓝色；
- 如果车辆贴墙滑移或卡住，truth 本身可能继续发生物理微动，此时误差不能简单归因于里程计。

## SLAM 接入方式

`slam_toolbox` 依赖 TF：

```text
map -> odom -> base_link -> livox_frame
```

因此要让 SLAM 使用组合里程计，关键不是只发布 `/localization/composite_odometry`，而是要让组合里程计成为唯一的 `odom -> base_link` TF 发布者。

当前 mapping 启动策略：

```text
joint_feedback_odometry:
  发布 /localization/odometry
  不发布 TF

composite_wheel_imu_odometry:
  发布 /localization/composite_odometry
  发布 odom -> base_link TF

slam_toolbox:
  odom_frame = odom
  base_frame = base_link
  scan_topic = /livox/scan
```

这样 SLAM 的 scan matching 会基于组合里程计提供的短时运动先验工作，而不是基于原始轮式 yaw。

## 当前边界

当前方案仍然不是全局定位方案：

- 组合里程计仍会随路径长度累计前向距离误差；
- 长时间贴墙滑移会污染轮速距离；
- IMU yaw 可以显著改善航向，但不能修正 XY 漂移；
- 后续全局一致性仍需要 SLAM scan matching 和回环闭合；
- Nav2 暂未接入该 odom 作为导航源，需要在 SLAM 验证稳定后再切换。

## 后续优化方向

优先级建议：

1. 在迷宫中用 direct keyboard 手操长路径，记录 wheel/composite/truth 误差；
2. 根据迷宫手操结果调整异常轮残差阈值和静止阈值；
3. 验证 SLAM 地图是否比旧 wheel TF 更少扭曲；
4. 若地图稳定，再让 Nav2 使用该组合里程计；
5. 最后再考虑引入 LiDAR scan matching / localization 作为漂移修正层。
