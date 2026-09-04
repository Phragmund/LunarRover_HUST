# LunarRover HUST

面向月球熔岩洞未知环境巡视、建图与自主采样任务的 ROS 2 Humble 工程。本仓库提供可复现的六轮摇臂月球车仿真、定位建图、地形导航与目标操作软件基础。

## 能力范围

- Gazebo Fortress 六轮摇臂车仿真、Mid-360 与 Gemini 335 传感器模拟；
- 轮速/IMU 局部里程计与 FAST-LIO2 全局定位；
- 基于注册点云的二维地形可通行性图；
- Nav2 导航、前沿探索、人工接管和底盘安全命令仲裁；
- 面向已知目标的视觉观测、三维定位和机械臂执行接口。

真实硬件、第三方推理模型和视觉抓取均须按其依赖条件单独配置；未经实车验证的模块不得作为已完成能力对外宣称。

## 架构

```text
Livox / Gemini / 仿真传感器
  -> localization (FAST-LIO2) -> mapping (terrain map)
  -> navigation (Nav2) -> control -> chassis
                              ^
                         exploration

camera -> manipulation -> arm / gripper
```

包职责、运行边界见 [`src/README.md`](src/README.md)。唯一的 ROS 接口定义和使用规则见 [`docs/interface_contract.md`](docs/interface_contract.md)。

## 环境与构建

目标环境为 Ubuntu 22.04、ROS 2 Humble 与 Gazebo Fortress。先导入固定依赖，再构建：

```bash
source /opt/ros/humble/setup.bash
./scripts/import_dependencies.sh
./scripts/import_hardware_dependencies.sh
./scripts/build.sh
source install/setup.bash
```

第三方依赖来源在 `dependencies/*.repos`；FoundationPose 与其权重不随仓库分发。实机默认确认器为随仓库交付的自训练 Ultralytics 检测权重 `weights/best.pt`；部署前须按 [`docs/yolo_training_reproducibility.md`](docs/yolo_training_reproducibility.md) 完成验收。

## 提交前验证

```bash
make validate
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

提交时应附上固定世界、初始位姿、参数文件、运行日志/rosbag 和结果指标，区分仿真验证与实车验证。

## 目录

- `src/`：ROS 2 包；
- `scripts/`：安装、构建和受控启动脚本；
- `dependencies/`：外部源码固定版本清单；
- `docs/`：技术方案、接口契约和验证说明；
- `tools/`：检查、标定与数据分析工具。
- `debug/`：开发期调试图和中间结果，不作为正式提交证据。

`build/`、`install/`、`log/`、`.deps/`、权重和运行数据均为本地生成物，不属于源码交付内容。
