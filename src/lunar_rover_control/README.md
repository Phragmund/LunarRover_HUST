# lunar_rover_control

This package contains only the low-level simulation-facing vehicle interfaces:

- `/chassis/cmd_vel` input as `geometry_msgs/msg/TwistStamped`;
- four steering position commands;
- six wheel velocity commands;
- passive rocker differential control;
- 5-DOF arm trajectory control (`arm_controller`, J1-J4);
- gripper position control (`gripper_controller`, J5);
- simulated `/chassis/state` feedback and command watchdog.

It intentionally contains no wheel odometry, pose estimation, filtering,
mapping, planning, obstacle avoidance, or autonomous mission logic.

## Arm topics

| Controller | Type | Topic | Joints |
| --- | --- | --- | --- |
| `arm_controller` | `joint_trajectory_controller/JointTrajectoryController` | `/arm_controller/joint_trajectory` | `arm_joint1` .. `arm_joint4` |
| `gripper_controller` | `position_controllers/JointGroupPositionController` | `/gripper_controller/commands` (`std_msgs/msg/Float64MultiArray`) | `arm_joint5` |

## Keyboard arm joint controller

`scripts/keyboard_arm_joint_controller.py` 复用 `lunar_rover_navigation` 直读
`/dev/input/event*` 的键盘后端，用于手动调试机械臂关节：

```bash
ros2 run lunar_rover_control keyboard_arm_joint_controller.py
```

按键：

| 按键 | 功能 |
| --- | --- |
| 小键盘 `1`-`5`（或主键盘 `1`-`5`） | 选择 `arm_joint1` .. `arm_joint5` |
| `Up` / `Down` | 当前关节正 / 反方向连续运动 |
| `Left` / `Right` | 切换到上一个 / 下一个关节 |

参数：`arm_command_topic`（默认 `/arm_controller/joint_trajectory`）、
`gripper_command_topic`（默认 `/gripper_controller/commands`）、
`joint_state_topic`（默认 `/joint_states`）、`step_rates`、`joint_limits`。
运行前需确保用户可读 `/dev/input/event*`（参见 `docs/keyboard_teleop_direct.md`）。

## Keyboard arm IK controller

`scripts/keyboard_arm_ik_controller.py` 将统一调试台的 IK 手操方式移植到
ROS 侧。节点只发布 ROS 机械臂控制命令，不直接写 H7 串口；实车链路下仍由
`motion_solver.py` 把轨迹目标转换成 `$ARM:MULTI`。

```bash
ros2 run lunar_rover_control keyboard_arm_ik_controller.py
```

默认使用实测机械臂几何参数：

| 参数 | 默认值 |
| --- | --- |
| `base_to_shoulder_m` | `0.07545` |
| `upper_arm_m` | `0.11391` |
| `forearm_m` | `0.11000` |
| `tool_m` | `0.15601` |
| `home_degrees` | `[96.754752, 201.804458, 183.510522, 110.293936, 45.0]` |

`home_degrees` 是 2026-08-29 实车视觉抓取测试时手动摆出的朝向桌面的 IK 初始位，
`R` 键会回到该位姿。

目标位姿坐标定义为机械臂基座坐标系：

| 轴 | 方向 |
| --- | --- |
| `+X` | 车体正前方 |
| `+Y` | 车体左方 |
| `+Z` | 向上 |

按键：

| 按键 | 功能 |
| --- | --- |
| `W` / `S` | 末端目标前 / 后平移 |
| `A` / `D` | 末端目标左 / 右平移 |
| `Space` / `Shift` | 末端目标上 / 下平移 |
| `Up` / `Down` | 末端姿态 pitch 增 / 减 |
| `Right` / `Left` | 夹爪打开 / 闭合 |
| `R` | 回到 IK 初始位 |

实车默认通过 topic 手操接口控制，连续键盘控制更接近统一调试台的手感：

```bash
ros2 run lunar_rover_control keyboard_arm_ik_controller.py --ros-args \
  -p command_mode:=topic
```

需要和 MoveIt/action 语义一致时可改为：

```bash
ros2 run lunar_rover_control keyboard_arm_ik_controller.py --ros-args \
  -p command_mode:=action
```
