# lunar_rover_localization

FAST-LIO2 适配与 TF 桥接包。

```text
/livox/points + /livox/imu -> FAST-LIO2
  -> /localization/lio_odometry -> map_odom_tf_bridge -> map -> odom
  -> /localization/cloud_registered
```

`fast_lio2_map_frame_node.py` 规范化外部 FAST-LIO2 输出；`map_odom_tf_bridge.py` 连接全局 LIO 与局部里程计。此包不使用 Gazebo 真值作为正式输出。配置位于 `config/`，运行接口以 [`../../docs/interface_contract.md`](../../docs/interface_contract.md) 为准。

`launch/fast_lio2_localization.launch.py` 是定位组合入口；`scripts/` 同时保留输入归一化、仿真 IMU 适配、后向点云过滤和仅用于调试的真值节点。正式链路不得启用真值替代节点。
