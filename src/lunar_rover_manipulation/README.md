# lunar_rover_manipulation

目标观测、三维定位、机械臂动作协调与任务执行包。`real_grasping.launch.py` 可组合检测、定位、抓取协调和任务节点；实际启用的后端由 launch 参数和外部依赖决定。

自动执行前必须满足：目标 `pose_is_precise=true`、TF 与时间戳有效、置信度达到阈值、位姿位于工作空间且碰撞检查通过。该包只发布控制器动作目标，不直接写硬件串口。FoundationPose、YOLO 权重及 GPU 条件均为可选外部依赖，未安装时不得启用相应后端。

## 目录

- `launch/`：视觉抓取系统组合入口；
- `scripts/`：检测、定位、手眼采样、主动搜索、抓取协调、状态机和任务执行节点；
- `config/`：抓取阈值、MoveIt、目标数据集与 FoundationPose 对象配置；
- `srdf/`：机械臂语义模型；`test/`：感知与抓取几何测试。
