# lunar_rover_navigation

导航与底盘命令适配包。该包把 Nav2 输出转换为月球车底盘接口，并提供地形代价层和键盘接管。

## 主数据流

```text
/terrain/traversability
  -> TerrainLayer
  -> Nav2 planner/controller
  -> /cmd_vel
  -> twist_stamper_node
  -> /navigation/chassis_cmd_vel
  -> chassis_cmd_mux_node.py
  -> /chassis/cmd_vel
```

键盘接管输入为 `/keyboard/chassis_cmd_vel`。mux 在键盘输入新鲜时转发键盘命令，超时后自动恢复导航命令。

## 文件

| 路径 | 说明 |
| --- | --- |
| `launch/navigation.launch.py` | 启动 Nav2、生命周期激活节点、twist stamper 和 command mux。 |
| `launch/sim_mapping_navigation.launch.py` | 旧仿真 mapping + navigation 调试入口。 |
| `src/terrain_layer.cpp` | Nav2 costmap 插件，从 `/terrain/traversability` 写入代价地图。 |
| `src/twist_stamper_node.cpp` | 将 Nav2 `/cmd_vel` 适配为 `TwistStamped`，并做限速/近场防护。 |
| `scripts/chassis_cmd_mux_node.py` | 在导航和键盘控制之间选择最终 `/chassis/cmd_vel`。 |
| `scripts/keyboard_rover_teleop.py` | 直接读取 Linux `/dev/input` 的键盘控制节点。 |
| `scripts/nav2_lifecycle_bringup_node.py` | 等待并激活 Nav2 lifecycle 节点。 |
| `scripts/navigation_smoke_test.py` | 导航链路冒烟测试。 |
| `scripts/frontier_exploration_test.py` | 前沿探索相关测试辅助。 |
| `config/nav2_fast_lio2_params.yaml` | 当前 FAST-LIO2 地形导航参数。 |
| `config/nav2_mapping_params.yaml` | 旧 2D mapping 导航参数。 |
| `config/twist_stamper.yaml` | twist stamper 和 command mux 参数。 |

详见 [`../../docs/keyboard_teleop_direct.md`](../../docs/keyboard_teleop_direct.md)。
