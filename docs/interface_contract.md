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
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 | 仅为导航内部输出，不直连底盘。 |
| `/chassis/cmd_vel` | `geometry_msgs/msg/TwistStamped` | navigation/control | 底盘唯一 ROS 速度入口。 |
| `/chassis/state` | `lunar_rover_msgs/msg/ChassisState` | hardware/control | 底盘安全与故障状态。 |

## 控制权与安全

导航速度依次经过 `twist_stamper_node` 和 `chassis_cmd_mux_node`，再到 `/chassis/cmd_vel`。人工接管发布到其专用输入，由 mux 在输入新鲜时切换；任何节点不得直接同时向最终底盘话题发布。下位机必须实施 250 ms 命令超时、急停、故障锁止和独立限幅；完整要求见 [`lower_controller_contract.md`](lower_controller_contract.md)。

## 操作接口

| 接口 | 类型 | 约定 |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 原始彩色图。 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 原始深度图。 |
| `TargetObservation` | `lunar_rover_msgs/msg/TargetObservation` | `pose` 必须标注可解析的 frame；`pose_is_precise=false` 时禁止自动抓取。 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 控制 `arm_joint1` 至 `arm_joint4`。 |
| `/gripper_controller/commands` | `std_msgs/msg/Float64MultiArray` | 控制 `arm_joint5`。 |

机械臂节点只提交动作目标，不可绕过控制器直接写串口。目标检测、位姿估计和抓取规划均需在执行前验证 TF、时间戳、置信度、工作空间和碰撞约束。

## 外部 action

探索模块只通过 Nav2 `/navigate_to_pose` action 提交目标；它不直接发布速度。动作取消、超时或定位/地形输入失效时，应撤销目标并释放到底盘安全静止状态。
