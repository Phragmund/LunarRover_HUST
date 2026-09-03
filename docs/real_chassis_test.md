# 实车底盘链路预检与启动

当前实车手动控制链路以
[`lower_controller_contract.md`](lower_controller_contract.md)
为准：

```text
keyboard_controller / web_joystick_server
  -> /manual_control (lunar_rover_msgs/ManualControl)
  -> motion_solver
  -> H7 USB CDC
  -> 3508 + 舵轮 + 机械臂
```

历史的 `chassis_driver_node.py + chassis_cmd_mux_node.py + /chassis/cmd_vel`
链路仍保留给仿真/Nav 调试，不再作为当前实车手动控制入口。

## 上车前硬性条件

1. 车轮悬空或车辆四周清空，实体急停可立即触达。
2. H7 USB CDC 已连接，设备通常为 `/dev/ttyACM0`，重插后可能变为
   `/dev/ttyACM1`。
3. 手机与上位机处于同一网络。

## 无动作预检

```bash
cd /home/phragmund/LunarRover
REAL_CHASSIS_H7_PORT=/dev/ttyACM0 scripts/check_real_chassis.sh
```

预检只检查 ROS/Python 依赖、H7 串口权限和重复进程，不发送电机控制帧。

## 启动

```bash
cd /home/phragmund/LunarRover
REAL_CHASSIS_H7_PORT=/dev/ttyACM0 scripts/run_real_chassis.sh
```

脚本会启动 `motion_solver` 和带随机令牌的 `web_joystick_server`，终端会打印手机
访问 URL。

> 安全：`motion_solver` 启动即发送 `$M3508:ENABLE,1`。页面摇杆一推，3508 就会
> 响应。启动前必须确认车体架起或周围清空。

切换 Wi-Fi/热点后，需要重启 `motion_solver` 和 `web_joystick_server`。一键脚本
会重新打印当前网络下的手机访问地址。
