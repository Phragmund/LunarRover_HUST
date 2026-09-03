# lunar_rover_control

底层车辆与机械臂控制包，提供四轮转向、六轮驱动、摇臂差动、机械臂/夹爪控制、状态反馈和命令看门狗；不包含定位、建图、规划或任务决策。

| 输入/输出 | 类型 | 用途 |
| --- | --- | --- |
| `/chassis/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 底盘唯一 ROS 速度入口。 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 车轮、转向、摇臂和机械臂反馈。 |
| `/chassis/state` | `lunar_rover_msgs/msg/ChassisState` | 安全、故障和使能状态。 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | J1–J4 轨迹命令。 |
| `/gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` | J5 夹爪命令。 |

底盘安全要求与命令超时规则见 [`../../docs/lower_controller_contract.md`](../../docs/lower_controller_contract.md)。

## 目录

- `include/`、`src/`：四轮转向/六轮驱动运动学、控制器插件、仿真状态与轮反馈里程计；
- `config/`：控制器、实车串口、机械臂和里程计参数；
- `launch/`：仿真控制与实车控制入口；
- `scripts/`：串口、CAN、网页和键盘调试节点；
- `test/`：运动学、实车接口和机械臂映射测试。
