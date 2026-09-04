# ROS 2 包说明

工作区按“公共接口—模型/仿真—传感器/控制—定位建图—导航探索—任务操作—集成启动”分层。下层不依赖上层决策包；真值话题只用于验证，不可进入算法闭环。

| 包 | 职责 |
| --- | --- |
| `lunar_rover_msgs` | 公共消息：底盘状态、人工控制、目标观测。 |
| `lunar_rover_description` | URDF/Xacro、关节名、传感器与机械臂坐标系。 |
| `lunar_rover_simulation` | Gazebo 世界、模型、控制器与 ROS-Gazebo bridge。 |
| `lunar_rover_sensors` | Livox 与 Gemini 的仿真/实车原始传感器入口。 |
| `lunar_rover_control` | 底盘和机械臂底层控制、命令看门狗与状态反馈。 |
| `lunar_rover_localization` | FAST-LIO2 输入输出适配及 `map -> odom` TF。 |
| `lunar_rover_mapping` | 轮速/IMU 里程计、点云融合与地形图。 |
| `lunar_rover_navigation` | Nav2 地形层、速度适配与控制权仲裁。 |
| `lunar_rover_exploration` | 前沿目标选择及 Nav2 action 调度。 |
| `lunar_rover_manipulation` | 目标观测、定位、机械臂抓取与任务执行。 |
| `lunar_rover_bringup` | 仿真与实车系统级启动组合。 |

主数据流按部署区分：仿真/标准 ROS 使用 `/livox/* -> localization -> mapping -> navigation -> /chassis/cmd_vel`；实机使用 `/livox/* -> localization -> mapping -> navigation -> /manual_control -> motion_solver/H7`。操作支路为 `/camera/* -> manipulation -> arm_controller/gripper_controller`，任务层通过 mission executive 协调搜索、抓取、返航与投放。接口类型、frame 与控制权规则以 [`../docs/interface_contract.md`](../docs/interface_contract.md) 为准。
