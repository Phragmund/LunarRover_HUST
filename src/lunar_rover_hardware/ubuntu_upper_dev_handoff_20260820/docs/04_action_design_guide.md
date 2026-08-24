# 机械臂“动作设计”与动作文件教程

## 1. 这是什么

统一调试页 rover_web_controller.html 的机械臂页原来有两个模式：

1. 关节控制
2. 逆运动学

本次新增第三个模式：动作设计。它像一个视频剪辑 / 动画制作的时间轴，用五条轨道编排一段完整的机械臂动作，并可以：

- 现场“播放”，把动作按时间顺序发给实机；
- “回到起点”，让机械臂沿原路逆行回到起始姿态；
- 导出一份 .action.json 文件，交给上位机生成动作序列。

## 2. 界面与交互

进入机械臂页后，点右上角「动作设计」进入时间轴。

### 2.1 五条轨道

五条轨道自上而下对应五个舵机 j1..j5：

| 轨道 | 舵机 ID | 行程 | 说明 |
| --- | --- | --- | --- |
| J1 | 1 | 0..270° | 正常 |
| J2 | 2 | 0..270° | 反向 |
| J3 | 3 | 0..270° | 正常 |
| J4 | 4 | 0..270° | 反向 |
| J5 | 5 | 0..90° | 正常，夹爪 |

轨道就是时间轴，可以左右滑动（横向滚动）。

### 2.2 指针（播放头）

- 一条竖线指示当前时间轴位置，单位毫秒 ms。
- 可以直接拖动指针顶部把手。
- 键盘 ← / → 移动指针；按住 Shift 微调（20ms），按住 Ctrl/Alt 更细（10ms）。
- 键盘 ↑ / ↓ 切换选中的轨道，选中轨道高亮。

### 2.3 功能按钮

| 按钮 | 作用 |
| --- | --- |
| 插入动作帧 | 在当前选中轨道、指针所在时刻插入一条旋转指令（默认 500ms，目标 = Home 姿态） |
| 编辑动作帧 | 把下面“编辑区”里填好的目标位置/时长/起始时刻应用到选中的指令条 |
| 播放 | 指针沿时间轴正向运动，把扫过的指令条依次发到实机 |
| 回到起点 | 指令条逆行回起点（见 2.5），绝不直接发起点位置 |
| 停止 | 暂停/停止播放 |
| 导出动作文件 | 下载 .action.json |
| 清空全部 | 删除所有动作帧 |

编辑区支持「目标角度 / 原始 P / 消耗时间 ms / 起始时刻 ms」四个字段；选中指令条后填入、点「编辑动作帧」或「应用到选中」生效。

### 2.4 指令条（动作帧）

每条轨道上的彩色长条 = 一条“从 A 转到 B，耗时 T”的舵机指令：

- 横坐标 = 指令开始时刻；
- 长度 = 消耗时间（也就是 $ARM:MOVE 里的 time 参数）；
- 文字 = 目标位置（角度 + P 值）。

交互：

- 点击指令条选中它；
- 拖动指令条两端改时长（左端改起点，右端改终点）；
- 拖动指令条中间改起始时刻；
- 同一轨道上指令不重叠（插入/拖动会自动靠边）。

### 2.5 播放与“回到起点”的逆行语义

正向播放：指针扫到某条指令的起点时，立即发送该指令 $ARM:MOVE,id,pulse,time! ，time = 指令时长。这样舵机实际运动与指针视觉扫掠同步。

回到起点必须逆行：指针从当前位置往回扫，对每条指令反过来发一条“目标 → 起点”的 $ARM:MOVE，让它沿刚才的路径原路返回。

> 为什么要逆行而不是直接发 Home 位置？因为多关节联动时，直接跳到起点会让机械臂“抄近道”直线回到起始位，极可能与来时的轨迹不同、撞到障碍或卡住。逆行则保证它按原路退回。

正向/逆向之间有个隐含约定：动作的初始姿态 = Home 姿态（j1=90 j2=180 j3=135 j4=135 j5=45）。每条指令的“起点角度”由同轨道上一条指令的目标决定，第一条指令的起点 = Home。

## 3. 数据模型

一条动作帧 clip 的结构：

    {
      "id": "c1",
      "track": "j1",
      "servo_id": 1,
      "start_ms": 0,
      "duration_ms": 500,
      "from_angle": 90.0,
      "from_pulse": 1500,
      "target_angle": 120.0,
      "target_pulse": 1567
    }

字段含义：

- start_ms：指令开始时刻
- duration_ms：消耗时间 = $ARM:MOVE 的 time
- from_angle / from_pulse：起点角度（同轨道上一条的目标，首条 = home）
- target_angle / target_pulse：目标角度

角度 ↔ P 值换算（与网页一致）：

    ratio = (angle - min_deg) / (max_deg - min_deg)
    inverted 关节：ratio = 1 - ratio
    pulse = round(min_pulse + ratio * (max_pulse - min_pulse))

## 4. 导出文件格式

导出的 .action.json 顶层结构：

    {
      "format": "lunar-rover-arm-action",
      "version": 1,
      "meta": {
        "name": "arm_action",
        "exported_at": "2026-08-24T00:00:00.000Z",
        "time_unit": "ms",
        "home_angles": { "j1": 90, "j2": 180, "j3": 135, "j4": 135, "j5": 45 }
      },
      "joints": [
        { "name": "j1", "servo_id": 1, "min_deg": 0, "max_deg": 270,
          "min_pulse": 500, "max_pulse": 2500, "inverted": false }
      ],
      "timeline": { "duration_ms": 6000 },
      "clips": [ ... 见第 3 节，按 start_ms 升序 ... ],
      "sequence": {
        "play":    [ ... 正向指令，按 at_ms 升序 ... ],
        "reverse": [ ... 逆行指令，按 at_ms 降序 ... ]
      },
      "poses": [ ... 关键姿态序列，见 5.3 ... ],
      "note": "..."
    }

其中 sequence.play / sequence.reverse 的每一项形如：

    {
      "at_ms": 0,
      "track": "j1",
      "servo_id": 1,
      "command": "$ARM:MOVE,1,1567,500!",
      "from_pulse": 1500,
      "to_pulse": 1567,
      "duration_ms": 500
    }

command 已经是可直接发送的完整帧字符串，上位机最简单可以原样按序回放。

## 5. 上位机如何使用动作文件生成动作序列

下面给两种用法，从简单到正式。

### 5.1 直接串口回放（与网页行为一致，最简单）

前提：上位机独占 H7 的 USB CDC 串口（见 01_comm_control_guide.md）。

    #!/usr/bin/env python3
    # replay_action.py —— 把导出的 .action.json 原样回放到 H7
    import json
    import time
    import sys

    import serial


    def load_action(path):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        assert data["format"] == "lunar-rover-arm-action", "不是动作文件"
        return data


    def send(ser, frame):
        ser.write(frame.encode("ascii"))   # 帧以 ! 结尾，无需额外换行
        ser.flush()


    def play(ser, action, reverse=False):
        seq = action["sequence"]["reverse" if reverse else "play"]
        order = sorted(seq, key=lambda e: e["at_ms"], reverse=reverse)
        for ev in order:
            send(ser, ev["command"])
            # 舵机靠 command 里自带的 time 参数自行运动；
            # 如需严格对齐事件时间，可在此按 at_ms 差值 sleep。


    def main():
        action = load_action(sys.argv[1])
        ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.05)
        time.sleep(0.6)
        ser.reset_input_buffer()

        for jid in range(1, 6):
            ser.write(("$ARM:TORQUE,%d,1!" % jid).encode("ascii"))
        ser.flush()

        play(ser, action, reverse=False)   # 正向动作
        time.sleep(0.5)
        play(ser, action, reverse=True)    # 回到起点（逆行）

        ser.close()


    if __name__ == "__main__":
        main()

要点：

- 每条 command 已带 time（毫秒），H7/舵机板自己完成一段匀速旋转；上位机不需要按固定周期补发位置。
- 正向按 at_ms 升序发；逆行按 at_ms 降序发，且逆行命令的 from_pulse/to_pulse 已互换。
- 不要在逆行时直接发 meta.home_angles 作为终点，否则不是原路退回。

### 5.2 转成 ROS FollowJointTrajectory（与现有 motion_solver 集成）

本仓库 lunar_rover_control/scripts/motion_solver.py 已经暴露了：

- /arm_controller/follow_joint_trajectory
- /gripper_controller/follow_joint_trajectory

它内部会把轨迹点转成 $ARM:MULTI,id1,pulse1,...,time! 下发。动作文件可以这样转成轨迹：

1. 取 poses（关键姿态序列，每个点含五关节角度 + 相对 0 点的时间 t_ms）；
2. 相邻两个点的时间差作为该段的 time_from_start 增量；
3. 由于 $ARM:MULTI 只有一个 time，而动作设计里各关节时长可以不同，推荐按事件时间切段：
   - 把所有指令的 start_ms 与 start_ms + duration_ms 收集排序，得到事件时刻；
   - 每个事件时刻取 poses 中对应姿态作为轨迹点；
   - 段时长 = 相邻事件时刻差，并夹到 motion_solver 的 arm_min_segment_ms .. arm_max_segment_ms；
4. 把轨迹点组装成 trajectory_msgs/JointTrajectory 发给 action server。

伪代码：

    import json
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    def action_to_trajectory(path, joint_names):
        data = json.load(open(path, encoding="utf-8"))
        pts = data["poses"]
        traj = JointTrajectory()
        traj.joint_names = joint_names
        prev_t = None
        for p in pts:
            pt = JointTrajectoryPoint()
            pt.positions = [ _degrees_to_ros(n, p["angles"][n]) for n in joint_names ]
            dt = 0 if prev_t is None else (p["t_ms"] - prev_t)
            prev_t = p["t_ms"]
            pt.time_from_start.sec = int(dt // 1000)
            pt.time_from_start.nanosec = int((dt % 1000) * 1000000)
            traj.points.append(pt)
        return traj

### 5.3 状态机解读

poses 数组就是整个动作的“状态机”：

- 初始态（t_ms = 0）= meta.home_angles；
- 每到一个事件时刻，机械臂处于该时刻的完整五关节姿态；
- 相邻姿态之间的“转移”就是对应时段内若干舵机的旋转指令；
- 正向状态机沿 t_ms 递增走；逆向状态机沿 t_ms 递减走，且每条转移的起止角度互换、时长相同。

## 6. 注意事项

- 单串口独占：网页、h7_serial_client.py、CLI 脚本同一时刻只能有一个占用 H7 USB CDC。
- 整帧发送、整帧解析：所有帧以 ! 结尾，回放时不要中途拆帧。
- 位置模式是“单发目标，H7 自己保持到位”，不要持续重发同一目标。
- 机械臂 READ 高频容易触发超时，只做低频校验（约 ≤5 Hz）。
- 逆行必须逐条反向回放，不要直接跳 Home，否则会卡住。
