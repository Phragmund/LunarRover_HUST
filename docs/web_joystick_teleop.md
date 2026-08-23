# 手机网页摇杆遥控 (Web Joystick Teleop)

本文档说明如何用手机或笔记本浏览器遥控月球车底盘。该功能与键盘控制器处于同一
层级：手机发布到独立的 `/web/chassis_cmd_vel`，实体键盘发布到
`/keyboard/chassis_cmd_vel`。`chassis_cmd_mux_node` 按“手机 > 键盘 > 导航”
仲裁，两个手动源的默认超时均为 0.5 秒，避免并发发布者交错写同一话题。

## 控制模型（与键盘控制器不同）

| 输入 | 动作 | 底盘指令 |
| --- | --- | --- |
| 摇杆上/下 | 前进 / 后退 | linear.x = +max_linear / -max_reverse |
| 摇杆左/右 | 阿克曼转向 | 转向角 delta = -sx * ackermann_max_steering_angle，偏航角速度 wz = vx * tan(delta) / wheelbase |
| 按住「原地左转」 | 原地逆时针 | vx=0, wz=+spin_angular_speed |
| 按住「原地右转」 | 原地顺时针 | vx=0, wz=-spin_angular_speed |
| 「急停」 | 锁存停止 | 持续发布 0，直到点「解除急停」 |

关键点：摇杆横向是「转向角」而不是「偏航角速度」。杆量越大，转向角越大，
转弯半径 wheelbase / tan(delta) 越小；没有前进/后退速度时，横向杆量不产生
运动（原地转向交给两个按键）。实体键盘的 A/D 使用固定阿克曼角速度，Q/E
负责原地转向；手机摇杆则按横向杆量连续计算转向角。

## 依赖安装

在车机（上位机）上安装 Web 依赖（装进有 rclpy 的同一个 python3）：

    pip3 install --user "uvicorn[standard]" fastapi

## 构建

脚本注册在 lunar_rover_navigation 包的 install(PROGRAMS) 里，需要重新构建
一次让 ros2 run 能找到它：

    cd /home/phragmund/LunarRover
    ./scripts/build.sh

## 启动

    cd /home/phragmund/LunarRover
    scripts/run_web_joystick.sh

或直接：

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run lunar_rover_navigation web_joystick_server.py

不重新构建、临时快速验证时也可以直接跑源码：

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python3 src/lunar_rover_navigation/scripts/web_joystick_server.py

启动后终端会生成随机访问令牌并打印完整地址，例如
`http://192.168.1.23:8000/?token=...`。手机必须使用完整地址；可通过
`WEB_JOYSTICK_TOKEN` 固定令牌。

手机连上车机热点（或与车机同一 WiFi），浏览器打开该地址即可看到摇杆页面。

### 用 launch 一键启动（mux + 网页服务）

    ros2 launch lunar_rover_navigation web_joystick.launch.py \
      port:=8000 access_token:=请设置随机令牌

这会同时启动 chassis_cmd_mux 和 web_joystick_server。注意：如果导航链路
(navigation.launch.py) 已经在跑，mux 已存在，此时只需单独启动网页服务，
不要重复启动 mux。

## 参数

所有参数都能用 --ros-args -p key:=value 覆盖：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| command_topic | /web/chassis_cmd_vel | 手机独立手动控制话题 |
| port | 8000 | Web 服务端口 |
| max_linear | 0.16 | 前进最大速度 m/s |
| max_reverse | 0.10 | 后退最大速度 m/s |
| spin_angular_speed | 0.36 | 原地转向角速度 rad/s |
| ackermann_wheelbase | 0.52 | 阿克曼模型轴距 m |
| ackermann_max_steering_angle | 60 度 | 满杆转向角 |
| deadzone | 0.05 | 摇杆死区 |
| deadman_timeout | 0.5 | 无输入多少秒后自动停 s |
| idle_grace_period | 2.0 | 最后一个客户端断开后继续发零的时长 s |
| publish_rate | 50 | 指令发布频率 Hz |
| frame_id | base_footprint | TwistStamped 坐标系 |

## 安全设计

- 松手即停：摇杆松开时前端发零；按住期间每 100 ms 发送心跳，心跳中断
  0.5 秒后服务端强制发零。
- 断连即停：手机锁屏/切后台/断 WiFi 触发 WebSocket 断开，服务端停止接管，
  mux 的 5 秒超时后自动回到导航或停车。
- 急停锁存：急停按钮同时向 `/chassis/emergency_stop` 发布系统急停；清除
  急停后仍需单独向 `/chassis/controller_enabled` 发布 `true` 才能重新使能实车。
- 访问控制：WebSocket 默认由启动脚本生成的令牌保护。不要在实车上以空令牌启动。
- 服务端空闲不发布：没有客户端连接时服务端不发任何消息，不影响导航运行。

## 测试建议

先仿真后实车。仿真时加上 use_sim_time：

    ros2 run lunar_rover_navigation web_joystick_server.py --ros-args -p use_sim_time:=true

1. 起仿真 + chassis_cmd_mux，启动 web 服务。
2. 手机拖摇杆，ros2 topic echo /chassis/cmd_vel 核对符号：
   上推应为 vx>0，左推应为 wz>0（配合前进）。
3. 松手看是否立刻回零；断开 WiFi 看是否 0.5 秒内回零。
4. 点急停，确认持续发零且必须解除。
5. 实车先把 max_linear / max_reverse / spin_angular_speed 调小验证方向，
   确认无误再恢复默认值。
