# 下位机控制接口契约

本文档定义历史 `/chassis/cmd_vel` 实车控制接口，保留给仿真/Nav 兼容和后续自动
导航接入参考。

当前实车手动控制入口已经切换为
`/manual_control -> motion_solver -> H7 USB CDC`，详见
[`manual_control_and_motion_solver.md`](manual_control_and_motion_solver.md)。
不要把本文档中的 `chassis_driver_node.py + chassis_cmd_mux_node.py` 当作当前手动
上车启动链路。

历史实现由 NUC 上的 `chassis_driver_node.py` 完成 4 个动力轮和 4 个舵轮的解算与
闭环，再分别通过 USB2CANFD/CAN 和 USB CDC 下发；中间两轮为无动力从动轮。

## 上位机发布

### `/chassis/cmd_vel`

- 类型：`geometry_msgs/msg/TwistStamped`
- 建议频率：`20 Hz`
- QoS：`reliable`，`keep_last(1)`
- 坐标系：`base_footprint`
- 使用字段：`twist.linear.x` 和 `twist.angular.z`
- 未使用字段：`linear.y`、`linear.z`、`angular.x`、`angular.y` 必须按 `0` 处理

约定：

- `linear.x > 0` 表示车体前进方向。
- `angular.z > 0` 遵循 ROS REP-103，表示绕 Z 轴逆时针转向。
- 上位机安全监督节点已经做过限速和近场障碍刹停，但下位机仍必须独立执行最终限幅和急停。

## 下位机发布

### `/joint_states`

- 类型：`sensor_msgs/msg/JointState`
- 建议频率：`50 Hz` 或更高
- QoS：`sensor data` 或 `best_effort keep_last(10)`

必须包含以下关节名：

```text
left_front_steering_joint
right_front_steering_joint
left_rear_steering_joint
right_rear_steering_joint
left_front_wheel_joint
left_middle_wheel_joint
left_rear_wheel_joint
right_front_wheel_joint
right_middle_wheel_joint
right_rear_wheel_joint
left_rocker_joint
right_rocker_joint
left_bogie_joint
right_bogie_joint
```

`position` 必须对所有关节有效。`velocity` 至少必须对 6 个车轮有效，建议对转向、摇臂和摆臂也有效。单位使用 SI：弧度、弧度每秒。

### `/chassis/state`

- 类型：`lunar_rover_msgs/msg/ChassisState`
- 建议频率：`10 Hz`
- QoS：`reliable keep_last(10)`

字段语义：

- `mode`：`DISABLED`、`MANUAL`、`AUTONOMOUS` 或 `FAULT`
- `fault_bits`：下位机自定义故障位图，必须在联调文档中列出每一位含义
- `estop`：硬件或软件急停是否生效
- `enabled`：底盘驱动是否允许输出
- `watchdog_ok`：下位机是否认为上位机指令仍在超时时间内

## 下位机必须实现

- 4WS4WD 电机分配，包括前/后转向角和四个动力轮速度；中间轮从动。
- 电机电流、速度、位置和转向角限幅。
- 上位机指令超时保护：超过 `250 ms` 未收到 `/chassis/cmd_vel` 时输出零速度。
- 急停保护：急停后忽略非零速度指令，直到人工复位。
- 故障保护：严重驱动器、编码器、通信或电源故障时进入 `FAULT`。
- 时间戳：所有回传消息必须使用当前 ROS 时间或已同步系统时间。

## 历史上位机假设

- 历史 `/chassis/cmd_vel` 链路中，上位机不会在实车模式启动 `controller_manager`
  或 `ros2_control` 底盘控制器。
- 历史 `chassis_driver_node.py` 上电默认禁用，只有硬件在线、急停清除且收到
  `/chassis/controller_enabled=true` 后才允许非零输出。
- 历史链路使用 `/joint_states` 计算轮速里程计，并使用 `/chassis/state` 执行安全监督。
- 当前 `motion_solver` 手动链路启动即向 H7 发送 `$M3508:ENABLE,1`，安全语义见
  `manual_control_and_motion_solver.md`。
