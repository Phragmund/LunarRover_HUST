# 上位机手动控制与运动解算链路（交接文档）

> 本文档记录截至 2026-08 的开发状态：键盘 / 手机遥控 → 运动解算节点 → H7 下位机 的完整控制链路。
> 面向后续接手的上位机开发者，重点讲清楚「运动解算节点与各控制器链路的架构」以及「如何启动手机控制」。

---

## 1. 总览

整车上位机（Ubuntu / NUC）通过一个 H7 下位机的 USB CDC 虚拟串口控制底盘执行机构：

- 4 个 3508 行走电机（挂 H7 的 CAN1，ESC ID 1~4）
- 4 个舵轮转向电机（挂 H7 的 UART7）
- 机械臂舵机（挂 H7 的 UART10）

上位机只发两类命令：

- 动力轮：$M3508:SPD,s0,s1,s2,s3!（输出轴转速 r/s，正=前进，方向由下位机处理）
- 舵轮：$ANG,a0,a1,a2,a3!（角度 deg，正=顺时针、负=逆时针）

轮序统一为：0=右后、1=左后、2=右前、3=左前（与下位机一致）。

> 历史说明：本仓库早期还有「TwistStamped → chassis_cmd_mux → chassis_driver_node」的老链路，以及「达妙 USB2CANFD + dmcan」直连 CAN 的老后端。这些已不再用于手动控制，当前手动控制走的是下面这套新链路。老节点（chassis_driver_node / chassis_cmd_mux / keyboard_rover_teleop 等）仍保留在仓库里，用于仿真与 Nav 链路，勿与本文档的新链路混淆。

---

## 2. 系统架构

                         统一消息 /manual_control（lunar_rover_msgs/ManualControl）
                                           |
  keyboard_controller（键盘） ─────────────┤
  web_joystick_server（手机遥控） ─────────┤──► motion_solver（运动解算节点）──► H7 USB CDC 串口
  Nav 导航（预留，未接入） ────────────────┘          |
                                             $M3508:SPD + $ANG + $M3508:ENABLE + $STOP/$ZERO/$ARM:*

核心思想：三个控制源（键盘 / 手机 / Nav）只负责把自己转成同一条 ManualControl 消息（杆量 + 按键），由唯一的 motion_solver 节点独占 H7 串口、统一解算下发。这样新增控制源时，motion_solver 完全不用改。

- motion_solver 独占 H7 串口（/dev/ttyACM*）。
- 键盘、手机、Nav 都发布到 /manual_control（话题名可通过 input_topic 参数改）。

---

## 3. 消息定义：ManualControl

文件：src/lunar_rover_msgs/msg/ManualControl.msg

    int16 lx      # 左摇杆水平：右为正，阿克曼转向；±255 = 最大转向
    int16 ly      # 左摇杆垂直：前为正，油门；±255 = 最大速度
    int16 rx      # 右摇杆水平：非零触发原地转向（覆盖其他输入）；±255 = 四轮平均最大速度
    int16 ry      # 右摇杆垂直：预留，当前未使用
    bool relax    # R 键：断开所有转矩输出（单次触发）
    bool rearm    # T 键：重启位置保持 + 舵轮设零（单次触发）
    bool boost    # 左 Shift：倍率放大（motion_solver 里 boost_factor，当前 2.2）

杆量约定：

| 通道 | 语义 | 方向 |
| --- | --- | --- |
| LX | 阿克曼转向半径 | 右正（正=右转），|LX| 越大转向越急 |
| LY | 油门 | 前正（正=前进），±255 = 最大速度 |
| RX | 原地转向 | 非零即覆盖其他输入，正=右自转、负=左自转 |
| RY | 预留 | 未使用 |

---

## 4. 运动解算节点：motion_solver

文件：src/lunar_rover_control/scripts/motion_solver.py
节点名：motion_solver

### 4.1 职责

- 订阅 /manual_control。
- 独占 H7 串口，下发 $M3508:SPD（轮速）与 $ANG（舵角）。
- 处理 R（断开转矩）/ T（重启+设零）单次动作、boost 倍率。
- 优先级：RX 原地转 > LX 转向 > LY 直行 > 停止。

### 4.2 下发策略

- 轮速 SPD：周期发送（默认 20Hz，满足下位机 500ms 超时），由 H7SerialClient 内部循环负责。
- 舵角 ANG：单发（变化超过阈值才重发），满足「一次即可、下位机自持到位」，不反复补发。
- 使能：启动时发 $M3508:ENABLE,1；R 键 / 关闭时发 $M3508:ENABLE,0 + $M3508:STOP! + $STOP!。

### 4.3 解算逻辑

| 模式 | 触发 | 轮速 | 舵角 |
| --- | --- | --- | --- |
| 原地转 | RX ≠ 0 | 见「临时改动」 | 固定姿态「前内八后外八」[+a,-a,-a,+a]（a=spin_pose_deg），延迟 spin_delay 后启动轮 |
| 转向 | LX ≠ 0 | 见「临时改动」 | 见「临时改动」 |
| 直行 | LY ≠ 0 | 四轮同速 LY/255 × max_speed | 回正 0 |
| 停止 | 全 0 | 0 | 回正 0 |

原地转有 spin_delay（默认 0.5s）的「先摆舵、再启动轮」保护。

---

## 5. 控制器节点

### 5.1 keyboard_controller（键盘）

文件：src/lunar_rover_control/scripts/keyboard_controller.py
节点名：keyboard_controller

- 直接读 /dev/input/event* 键盘事件，映射成杆量 + 按键，发布 /manual_control（默认 20Hz + 按键变化即发）。
- 键位（杆量直接取 ±255 最大）：

| 键 | 映射 | 键 | 映射 |
| --- | --- | --- | --- |
| W | LY=+255（前进） | S | LY=-255（后退） |
| A | LX=-255（左转） | D | LX=+255（右转） |
| Q | RX=-255（左自转） | E | RX=+255（右自转） |
| R | relax（断开转矩） | T | rearm（重启+设零） |
| 左 Shift | boost | Space | 全杆量清零（停） |

### 5.2 web_joystick_server（手机遥控）

文件：src/lunar_rover_navigation/scripts/web_joystick_server.py
节点名：web_joystick_server

- FastAPI + uvicorn 起 Web 服务（默认端口 8000），手机浏览器打开网页，通过 WebSocket 回传摇杆/按键。
- 发布 /manual_control。
- 网页有：一个圆形摇杆（触控）+ 原地左/右转按钮 + 急停/解除急停 + 断开转矩(R)/重启设零(T)。
- 摇杆是方形映射：x/y 各自独立夹到 [-1,1]，四角可同时打到 (255,255)。
- 松手即停、切后台/断连自动停（有 deadman 超时）。

### 5.3 Nav 导航（预留）

尚未接入。将来导航节点同样发布 ManualControl 到 /manual_control 即可，motion_solver 无需改动。

---

## 6. H7 下位机协议（摘要）

完整协议见交接包：src/lunar_rover_hardware/ubuntu_upper_dev_handoff_20260820/docs/01_comm_control_guide.md

下行（上位机 → H7，单串口 115200 8N1）：

| 命令 | 说明 |
| --- | --- |
| $M3508:SPD,s0,s1,s2,s3! | 3508 速度模式，输出轴 r/s，正=前进 |
| $M3508:STOP! | 3508 停止 |
| $M3508:ENABLE,1/0! | 3508 使能/失能 |
| $ANG,a0,a1,a2,a3! | 舵轮角度模式，deg |
| $STOP! | 舵轮停止 |
| $ZERO! | 舵轮当前位置置零 |
| $ARM:TORQUE,id,1/0! | 机械臂关节扭矩开/关 |

上行遥测：$M3508:FB,...（3508 反馈）、$STD,...（舵轮反馈）。

下位机超时约束（必须遵守，否则下位机自动停）：

- 3508 速度命令超时 500ms（所以 SPD 要周期刷新）
- 舵轮位置命令超时 2s
- 舵轮速度命令超时 200ms

---

## 7. 启动步骤

### 7.0 前置条件（一次性）

1. 编译（改了消息要先重编 lunar_rover_msgs）：

        cd /home/phragmund/LunarRover
        source /opt/ros/humble/setup.bash
        colcon build --packages-select lunar_rover_msgs lunar_rover_control lunar_rover_navigation

2. 串口权限：仓库已放好 udev 规则 /etc/udev/rules.d/99-stm32-vcp.rules，把 STM32 VCP（0483:5740）自动设为 0666，重插 USB 会自动生效，无需手动 chmod。若失效，确认设备在 /dev/ttyACM* 且权限 crw-rw-rw-。

### 7.1 键盘控制

        cd /home/phragmund/LunarRover
        source /opt/ros/humble/setup.bash
        source install/setup.bash
        ros2 run lunar_rover_control motion_solver.py --ros-args -p h7_port:=/dev/ttyACM0

    另开一个终端：

        ros2 run lunar_rover_control keyboard_controller.py

> motion_solver 独占串口；键盘节点需 root 或 input 组权限读键盘。

### 7.2 手机控制（重点）

1. 插好 USB，确认设备号（可能是 ttyACM0，重插可能变成 ttyACM1，用 ls /dev/ttyACM* 查）。
2. 启动运动解算节点（用实际设备号）：

        source /opt/ros/humble/setup.bash
        source install/setup.bash
        ros2 run lunar_rover_control motion_solver.py --ros-args -p h7_port:=/dev/ttyACM0

3. 启动手机遥控服务（另开终端）：

        source /opt/ros/humble/setup.bash
        source install/setup.bash
        ros2 run lunar_rover_navigation web_joystick_server.py --ros-args -p port:=8000

4. 查上位机 IP：

        hostname -I

5. 手机连上位机所在网络（同一热点 / 同一 Wi-Fi），浏览器打开：

        http://<上位机IP>:8000

6. 页面右上角变「已连接」即通。操作：
   - 摇杆：上下=前进/后退，左右=转向（方形映射，四角可打满）。
   - ⟲/⟳ = 原地左/右自转（按住）。
   - ⛔ 急停 / ✅ 解除急停。
   - 🔌 断开转矩(R) / 🔄 重启设零(T)。

> ⚠️ 安全：motion_solver 启动即发 $M3508:ENABLE,1，摇杆一推轮子就动，启动前确认车体架起/周围清空。

### 7.3 网页独立调试（直连 H7，不走 ROS）

交接包自带的 Web Serial 调试页（不依赖 motion_solver，浏览器直接连 H7）：

        cd src/lunar_rover_hardware/ubuntu_upper_dev_handoff_20260820/tools/web
        python3 -m http.server 8088

浏览器（Chromium/Chrome）打开 http://<上位机IP>:8088/rover_web_controller.html，点「连接 H7 USB」选 ttyACM*。

> ⚠️ 网页（Web Serial）与 motion_solver（pyserial）不能同时占串口，只能二选一。切换前先停掉另一方。

---

## 8. 关键参数（motion_solver，可用 --ros-args -p 名:=值 覆盖）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| h7_port | /dev/ttyACM0 | H7 串口设备 |
| h7_backend | serial | serial / dummy |
| max_speed | 2.5 | 直行/转向的油门满量程速度（r/s） |
| max_steer_deg | 60.0 | 阿克曼最大舵角（原始解算用） |
| spin_pose_deg | 45.0 | 原地转固定姿态角 |
| spin_delay | 0.5 | 原地转先摆舵再启动轮的延迟（s） |
| boost_factor | 2.2 | 左 Shift 倍率 |
| arm_joint_ids | [1,2,3,4,5] | 机械臂关节 ID |

---

## 9. 当前临时改动与恢复（重要）

当前 motion_solver.py 里有几处临时调参改动，源码里都用「===== 临时更改 =====」「===== 原始… =====」注释框标出来了，恢复方式 = 删掉临时块 + 取消原始代码注释。

1. 原地转向四轮同速 2.0（_enter_spin）：
   - 临时：四轮同速 speed = |RX|/255 × 2.0。
   - 原始：按前/后轮到中心的距离比例分配（前轮快、后轮慢），已注释保留。

2. LX 差速转向（_enter_turn）：
   - 临时：LX 映射为左右轮速度差值比例（左轮 = 油门×(1+LX/255)、右轮 = 油门×(1-LX/255)），不转舵（差速/滑移转向）。
   - 原始：阿克曼转向（转舵 + 轮速按到 ICR 距离分配），已注释保留。

3. max_speed 2.5：由原来的 0.8 调大，属调参值（非「临时改动」注释框标记，但可随时改回）。

> 恢复时先看源码里的注释框，按提示操作即可。

---

## 10. 已知问题与注意事项

1. 重插 USB 后设备号会变（ttyACM0 → ttyACM1 …）。每次启动前 ls /dev/ttyACM* 确认，并把 h7_port 指到实际设备。
2. 切换网络（如换热点）会导致已运行的 ROS 节点 DDS 发现失联。切换网络后，需重启 motion_solver 和 web_joystick_server 才能重新互联。
3. USB 掉线会让 motion_solver 崩溃退出（串口写失败异常）。重新插好 USB 后重启即可。
4. 手机网页改动后要重启 web_joystick_server（HTML 是嵌在 Python 源码里的字符串，运行中的进程不会热更新）；浏览器侧必要时硬刷新清缓存。
5. 网页 Web Serial 与 ROS 串口互斥（见 7.3）。
6. 舵轮的「取消位置保持」用的是 $STOP!（交接包舵轮命令里没有单独松舵指令），若实车发现 $STOP! 是「0 速保持」而非「释放转矩」，需改用四路板透传指令。
7. 转向方向符号：下位机约定 $ANG 正=顺时针（向右）、负=逆时针（向左）；运动学里「逆时针为正」所以下发前取反。
