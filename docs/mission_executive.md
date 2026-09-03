# 十分钟采集—返航投放任务调度

`mission_executive_node.py` 是整机任务层；它不绕过 FAST-LIO2、TerrainLayer、Nav2 或视觉抓取状态机，而是以定时器监督它们的状态转换。节点启动后保持 `READY`，不会自行驱动车体。通过下面命令显式开始一次任务：

```bash
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: start}"
```

状态流为：

```text
READY -> SEARCHING -> TARGET_PENDING -> GRASPING -> SEARCHING
                                      -> RETURNING -> UNLOADING -> COMPLETE
```

- `SEARCHING`：发布 `/mission/exploration_enabled=true`，前沿探索使用 FAST-LIO2/地形图和 Nav2 行进；RGB-D 与 HSV/YOLO 仍持续观察。
- `/search/target_confirmed` 到达时：取消 Nav2 目标、暂停前沿探索，触发 `/grasp/trigger`。抓取状态机在该阶段保持底盘停止，并自行执行目标居中、0.1 m 步进接近、工作区判定与视觉抓取。
- `/grasp/result=SUCCESS` 才增加 `/mission/collected_count`，然后恢复前沿探索；`UNCONFIRMED`、失败或超时不会计数。
- 每 0.25 s 计算 `已用时间 + 当前位置到记录起点的距离 / return_speed_mps + 转向余量 + 120 s 卸货余量 + 安全余量`。达到 600 s 约束时，停止探索并发送起点的 Nav2 返航目标。
- 到起点后发送 `/grasp/command=unload`。抓取节点用 `unload_arm_degrees` 移到投放姿态并打开夹爪，发布 `UNLOAD_COMPLETE` 后任务结束。

`/mission/autonomy_active` 直接使 Nav2 命令穿过 `nav_to_manual_mux`；网页/手柄的非零输入仍会优先接管。任务取消使用：

```bash
ros2 topic pub --once /mission/command std_msgs/msg/String "{data: abort}"
```

投入实机前，应在无物块条件下验证 `unload_arm_degrees` 是收纳箱上方安全姿态。任务调度不会在收到 `start` 前发出移动或机械臂指令。
