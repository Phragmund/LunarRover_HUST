# lunar_rover_bringup

系统级启动组合包，不承载算法实现。`bringup.launch.py` 选择 `sim` 或 `real` 模式并透传基础参数；定位、建图、导航、探索和操作功能分别由对应包启动。

仿真启动示例：

```bash
ros2 launch lunar_rover_bringup bringup.launch.py mode:=sim world:=reachable_maze
```

调用方应在传感器、TF 和定位输出就绪后再启动上层模块。参数和接口以 [`../../docs/interface_contract.md`](../../docs/interface_contract.md) 为准。

## 目录

- `launch/`：基础仿真/实车入口，以及雷达建图和全链路组合入口；
- `lunar_rover_bringup/`、`resource/`、`setup.py`：ament Python 包安装定义；
- `package.xml`：运行时依赖声明。
