# lunar_rover_manipulation

目标观测、三维定位、机械臂动作协调与任务执行包。`real_grasping.launch.py` 可组合检测、定位、抓取协调和任务节点；实际启用的后端由 launch 参数和外部依赖决定。

状态机后端的当前抓取闸门为：YOLO（`weights/best.pt`）确认后先用锁定 mask 做视觉居中，再对 mask 区域采集一个姿态的 5 组 RGB-D 距离；随后将 `TargetObservation` 通过活动 `base_link -> arm_base_link` TF 转换，并用 IK 解/FK 回算误差判断夹爪工作中心是否可达。不可达时底盘前进一个受控步长，停车后重复视觉居中、距离采样和 TF/IK 判定；只有确认可达才进入精细抓取。旧版固定半径工作区与 YOLO 后四角采样目前不在生产路径中。该包只发布控制器动作目标，不直接写硬件串口。FoundationPose、YOLO 权重及 GPU 条件均为可选外部依赖，未安装时不得启用相应后端。

## 目录

- `launch/`：视觉抓取系统组合入口；
- `scripts/`：检测、定位、手眼采样、主动搜索、抓取协调、状态机和任务执行节点；
- `config/`：抓取阈值、MoveIt、目标数据集与 FoundationPose 对象配置；
- `srdf/`：机械臂语义模型；`test/`：感知与抓取几何测试。
