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
