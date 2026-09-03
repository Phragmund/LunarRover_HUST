# lunar_rover_navigation

Nav2 地形导航与速度适配包。

```text
/terrain/traversability -> TerrainLayer -> Nav2 /cmd_vel
  -> twist_stamper_node -> /navigation/chassis_cmd_vel
  -> chassis_cmd_mux_node -> /chassis/cmd_vel
```

键盘、网页或其他人工输入只能发布到 mux 的专用输入；不得绕过 mux 直接与导航竞争 `/chassis/cmd_vel`。`TerrainLayer`、速度限制、近场安全限制和具体仲裁超时由 `config/` 参数声明。

`launch/` 提供导航、网页遥控和旧仿真组合入口；`src/`、`include/`、`plugins/` 实现并注册 TerrainLayer 与 TwistStamped 适配器；`scripts/` 包含命令仲裁、生命周期管理、手动输入、路径跟随和冒烟测试；`routes/` 存放预定义路线数据。
