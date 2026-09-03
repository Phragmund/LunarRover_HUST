# lunar_rover_exploration

地形感知前沿探索包。节点读取 Nav2 costmap、TF 和 `/navigate_to_pose` action server，选择可达的未知区域边界目标并提交 action。

它不直接发布底盘速度；目标取消、超时或输入失效时必须停止调度并交由导航/底盘安全链路处理。参数位于 `config/frontier_explorer*.yaml`。

`launch/` 提供节点入口，`scripts/frontier_explorer_node.py` 实现前沿聚类和目标评分，`config/` 区分仿真与实车参数。
