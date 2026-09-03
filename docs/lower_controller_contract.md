# 底盘下位机接口声明

本文是 [`interface_contract.md`](interface_contract.md) 的下位机实现补充。下位机必须把安全约束作为最终裁决，不能依赖上位机限幅。

## 命令输入

`/chassis/cmd_vel` 的类型为 `geometry_msgs/msg/TwistStamped`，坐标系为 `base_footprint`：`linear.x > 0` 前进，`angular.z > 0` 左转。未使用的 Twist 字段必须为零。

- 上位机建议以 20 Hz 发布；
- 下位机在 250 ms 未收到新命令时必须输出零速度；
- 急停、驱动器/编码器/通信/供电故障必须阻止非零输出，且人工复位前不得自动恢复；
- 所有速度、转向角、电流和位置必须在下位机独立限幅。

## 状态输出

`/joint_states` 使用 `sensor_msgs/msg/JointState`，应不低于 50 Hz，位置单位为 rad、速度单位为 rad/s。必须包含四个转向关节、六个车轮、左右 rocker 和左右 bogie，名称以 `lunar_rover_description` 的 URDF 为准。

`/chassis/state` 使用 `lunar_rover_msgs/msg/ChassisState`，建议 10 Hz；`mode`、`fault_bits`、`estop`、`enabled` 与 `watchdog_ok` 必须反映实时状态。`fault_bits` 的具体位定义必须随固件版本发布。

## 控制权

导航、键盘、网页或人工控制均不得绕过命令仲裁与安全看门狗。实车串口协议、CAN SDK 和设备路径属于硬件部署配置，不作为 ROS 公共接口。
