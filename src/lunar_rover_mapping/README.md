# lunar_rover_mapping

建图与局部里程计包。主链路将 FAST-LIO2 注册点云转换为地形栅格：

```text
/localization/cloud_registered -> terrain_mapper_node.py -> /terrain/traversability
```

可选的 Gemini 深度点云融合在输入缺失、超时或 TF 不可用时必须退化到雷达点云而不中断地形输出。`/localization/composite_odometry` 提供 `odom` 系局部里程计；`/terrain/traversability` 是导航输入，未知为 `-1`、墙体为 `100`。配置与测试分别位于 `config/`、`test/`。

`launch/terrain_mapping.launch.py` 为当前地形链路入口；`scripts/` 包含点云融合、地形栅格化和局部里程计；`src/` 与 `rviz/` 提供 elevation/RViz 插件和可视化配置。`mapping.launch.py`、`sim_mapping.launch.py` 仅保留作旧方案调试，不能与主链路混淆。
