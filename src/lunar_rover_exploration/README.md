# lunar_rover_exploration

地形感知前沿探索包。该包读取 Nav2 costmap/TF，选择未知区域边界附近的可达目标，并通过 Nav2 `NavigateToPose` action 调度探索。

## 文件

| 路径 | 说明 |
| --- | --- |
| `launch/frontier_exploration.launch.py` | 启动 `frontier_explorer_node.py`。 |
| `scripts/frontier_explorer_node.py` | 前沿聚类、目标打分、路径检查、失败黑名单和返航风险控制。 |
| `config/frontier_explorer.yaml` | 前沿大小、目标代价、信息增益、地形风险、返航风险和超时参数。 |

## 输入输出

- 输入：Nav2 costmap、`map -> base_link` TF、Nav2 action server。
- 输出：`/navigate_to_pose` action 目标。

探索模块不直接发布底盘速度；底盘控制权仍由 Nav2、`twist_stamper_node` 和 `chassis_cmd_mux_node.py` 管理。
