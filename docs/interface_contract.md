# LunarRover ROS 接口使用声明

本文件是跨包接口的唯一规范。话题名称、消息类型、坐标系、控制权和安全边界发生变化时，必须同步更新本文件、相关 launch、参数文件和测试。

## 坐标与时间

- 采用 ROS REP-103：`base_link` 的 X 前、Y 左、Z 上；角速度正方向遵循右手系；
- 全局链：`map -> odom -> base_link`；机器人模型和传感器外参由 URDF 发布；
- 仿真使用 `/clock`，实车使用系统时间；同一运行链不得混用两类时间；
- `/simulation/ground_truth/*` 仅限评估、标定和回归测试，禁止作为正式定位、建图、导航或任务决策输入。

## 核心数据接口

| 接口 | 类型 | 发布者 | 使用规则 |
| --- | --- | --- | --- |
| `/livox/points` | `sensor_msgs/msg/PointCloud2` | sensors/simulation | FAST-LIO2 原始雷达输入。 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | sensors/simulation | 与点云使用同一时间基准。 |
| `/joint_states` | `sensor_msgs/msg/JointState` | control/hardware | 使用 URDF 定义的关节名和 SI 单位。 |
| `/localization/lio_odometry` | `nav_msgs/msg/Odometry` | localization | 全局定位输出，`frame_id=map`。 |
| `/localization/composite_odometry` | `nav_msgs/msg/Odometry` | mapping | 局部里程计，`frame_id=odom`。 |
| `/localization/cloud_registered` | `sensor_msgs/msg/PointCloud2` | localization | 建图用注册点云。 |
| `/terrain/traversability` | `nav_msgs/msg/OccupancyGrid` | mapping | 地形图；未知值为 `-1`，墙体为 `100`。 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2（仿真/默认配置） | 导航内部输出，不直连底盘。 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2（实机配置） | 实机 Nav2 输出；经 `twist_stamper_node` 转换。 |
| `/navigation/chassis_cmd_vel` | `geometry_msgs/msg/TwistStamped` | navigation | 带时间戳、限幅和近场安全检查的导航速度。 |
| `/chassis/cmd_vel` | `geometry_msgs/msg/TwistStamped` | chassis_cmd_mux | 仿真/标准 ROS 底盘速度入口。 |
| `/manual_control` | `lunar_rover_msgs/msg/ManualControl` | nav_to_manual_mux / 人工控制 | 实机 H7 `motion_solver` 的最终上位机控制入口。 |
| `/chassis/state` | `lunar_rover_msgs/msg/ChassisState` | hardware/control | 底盘安全与故障状态。 |

## 控制权与安全

仿真/标准 ROS 链为 `Nav2 /cmd_vel -> twist_stamper_node -> /navigation/chassis_cmd_vel -> chassis_cmd_mux_node -> /chassis/cmd_vel`。实机链为 `Nav2 /cmd_vel_nav -> twist_stamper_node -> /navigation/chassis_cmd_vel -> nav_to_manual_mux -> /manual_control -> motion_solver/H7`；网页或键盘的非零人工输入可抢占自治输出。任何节点不得绕过所属链路的仲裁器直接同时向最终控制话题发布。下位机必须实施 250 ms 命令超时、急停、故障锁止和独立限幅；完整要求见 [`lower_controller_contract.md`](lower_controller_contract.md)。

## 操作接口

| 接口 | 类型 | 约定 |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 原始彩色图。 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 原始深度图。 |
| `TargetObservation` | `lunar_rover_msgs/msg/TargetObservation` | `pose` 必须标注可解析 frame。`pose_is_precise=true` 表示 FoundationPose 等精确结果并优先使用；普通 RGB-D 位姿可在 YOLO/HSV 锁定、时间戳、置信度、位姿离散度和工作区门限均满足时进入几何抓取。 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 控制 `arm_joint1` 至 `arm_joint4`。 |
| `/gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` | 控制 `arm_joint5`。 |
| `/search/target_confirmed` | `lunar_rover_msgs/msg/TargetObservation` | 由默认自训练 YOLO `weights/best.pt` 确认，并经连续观测、空间离散度和去重检查后的候选目标；任务层据此暂停探索并交接抓取。 |
| `/grasp/result` | `std_msgs/msg/String` | 抓取结果：`SUCCESS`、`UNCONFIRMED`、`FAILED`、`ABORTED` 或 `UNLOAD_COMPLETE`；只有 `SUCCESS` 可增加任务物块计数。 |
| `/mission/command` | `std_msgs/msg/String` | 任务显式控制：`start`、`return_home`、`abort`；节点启动时保持 `READY`，不得自行行驶。 |
| `/mission/state`、`/mission/status` | `std_msgs/msg/String` | 任务阶段和可读状态。 |
| `/mission/autonomy_active`、`/mission/exploration_enabled` | `std_msgs/msg/Bool` | 分别授予 Nav2 控制权、启停前沿探索。 |
| `/mission/collected_count` | `std_msgs/msg/UInt32` | 已成功抓取并计数的物块数量。 |

机械臂节点只提交动作目标，不可绕过控制器直接写串口。目标检测、位姿估计和抓取规划均需在执行前验证 TF、时间戳、置信度、工作空间和碰撞约束。

## 外部 action

探索模块只通过 Nav2 `/navigate_to_pose` action 提交目标；它不直接发布速度。动作取消、超时或定位/地形输入失效时，应撤销目标并释放到底盘安全静止状态。
