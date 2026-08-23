# 实车底盘链路预检与启动

当前链路为：键盘/手机/Nav2 -> `chassis_cmd_mux` -> `/chassis/cmd_vel` ->
`chassis_driver_node` -> USB2CANFD/CAN（4 个 M3508/C620 动力轮）和 USB CDC ->
STM32F407 -> 四路舵电机板。中间两轮为无动力从动轮。

## 上车前硬性条件

1. 车轮悬空或车辆四周清空，实体急停可立即触达。
2. USB2CANFD 使用出厂固件、Classic CAN 1 Mbps、120R 终端打开；四个 C620
   反馈 ID 必须为 `0x201` 到 `0x204`。
3. STM32 工程重新构建并烧录。仓库现有 `Lunarover_f407.hex` 早于新增的
   200 ms USB 命令看门狗，不能代表当前源码。
4. 舵轮摆到约定机械零位后上电。USB 驱动会等待四路反馈在线，再以当前位置
   执行一次 `$ZERO!`；任一路不在线都会拒绝启动。

## 无动作预检

插好 USB2CANFD 和 STM32 CDC 后运行：

```bash
cd /home/phragmund/LunarRover
scripts/check_real_chassis.sh
```

它只检查 ROS/Python 依赖、CAN SDK、动态库、设备枚举、串口权限和重复进程，
不会发送电机控制帧。所有项目必须为 `PASS`。

## 启动

```bash
cd /home/phragmund/LunarRover
scripts/run_real_chassis.sh
```

脚本会启动底盘驱动、命令仲裁和带随机令牌的手机遥控服务。CAN 运行库路径在
进程启动前注入，避免 ROS 先加载 Ubuntu 22.04 的旧 `libstdc++` 后导致 CAN SDK
报 `GLIBCXX_3.4.32 not found`。

底盘初始为禁用。先检查：

```bash
ros2 topic echo /chassis/state
```

只有 `fault_bits: 0`、四个 CAN 电机反馈在线、四个舵机反馈在线时才能使能：

```bash
ros2 topic pub --once /chassis/controller_enabled std_msgs/msg/Bool '{data: true}'
```

急停和清除命令：

```bash
ros2 topic pub --once /chassis/emergency_stop std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /chassis/emergency_stop std_msgs/msg/Bool '{data: false}'
```

清除急停不会自动恢复动力，必须再次显式使能。首次测试先不用 Shift 加速，并逐一
验证前进方向、左右原地转、四个舵角方向以及松手 250 ms 内动力轮归零。

## 故障位

`/chassis/state.fault_bits` 的高位由上位机保留：

- bit 28：CAN 控制线程或 USB2CANFD 发送故障；
- bit 29：至少一个 C620 反馈超过 200 ms；
- bit 30：USB CDC/舵机反馈故障；
- 其余位来自 `/chassis/fault_bits` 外部故障输入。
