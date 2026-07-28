# LunarRover 熔岩洞自主探测、建图导航与采样实施方案指导文档

本文档基于 `docs/project_info/` 中的赛题说明、答疑材料、场地图片、往届经验和当前仓库已有硬件接口重写。与“规则迷宫导航”不同，本项目真实目标是：机器人在未知月球熔岩洞模拟场地内自主进入、探索、建图、避障/越障、识别科学目标、抓取携带、返回基地并卸载。

本文只把已有研究成果和成熟开源路径作为工程基础，不把未经验证的想法作为主路线。后续实现应按本文的阶段逐项落地、测试和迭代。

## 1. 任务与约束摘要

正式任务闭环：

```text
非接触启动
→ 离开基地
→ 进入熔岩洞
→ 未知环境建图探索
→ 避障/越障
→ 搜索科学目标
→ 目标识别与三维定位
→ 机械臂/收集机构取样
→ 携带目标返航
→ 卸载到基地 0.5 m × 0.5 m 区域
```

来自项目资料的关键场地约束：

- 熔岩洞直径约 `1.5 m ± 0.1 m`；
- 地面通道宽度约 `1.0 m ± 0.1 m`；
- 入口约 `6°`、`1 m` 下坡，洞内大部分基本平坦；
- 随机障碍高度 `< 10 cm`，无人工标识，可被推动；
- 目标物 `3–10` 个，方木块或特殊形状人造物，约 `5 cm` 立方体包络，单个质量约 `300 g` 以内；
- 目标随机放置在地面和 `50 cm` 以下岩壁/岩壁平台，不嵌入；
- 熔岩洞内无人工光源，比赛光照不动态变化；
- 每次任务 `600 s`，两次出发机会均计分；
- 除远程启动外，不应与外界通信；若外部算力参与完赛，需要固定在机器人本体上。

当前仓库与硬件基础：

- 六轮摇臂结构月球车，当前 URDF 尺寸约 `650 mm × 500 mm`；
- Livox Mid-360 点云 `/livox/points`；
- Mid-360 IMU `/livox/imu`；
- 仿真 LaserScan `/livox/scan`；
- Gemini RGB-D：`/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/depth_registered/points`；
- 底盘速度接口 `/chassis/cmd_vel`，类型 `geometry_msgs/msg/TwistStamped`；
- 仿真真值 `/simulation/ground_truth/odometry` 只能用于评估，不能进入算法闭环。

## 2. 总体技术判断

这个项目不能按普通 2D 迷宫车处理。真实场地包含非规则岩壁、低照度、窄通道、沙地、低矮石块、坡面/坑洼和机械臂采样。因此主线应升级为：

```text
LiDAR-IMU 主定位
→ 2.5D/3D 地形地图
→ 未知环境探索导航
→ RGB-D 目标识别与三维定位
→ 车体视觉伺服
→ 机械臂/收集机构抓取
→ 任务状态机
→ 返航卸载
→ 自动回归验证
```

工程原则：

1. 主定位优先使用 LiDAR-IMU。洞内低光、岩壁纹理重复、沙地滑移明显，纯视觉或纯轮速不适合作为主定位。
2. 导航地图不能只有 2D 占据。必须表达高度、坡度、粗糙度、未知风险、正障碍和负障碍。
3. 探索目标不是完整覆盖地图，而是在 600 s 内“找到目标、拿到目标、回得来”。
4. 目标识别要语义和几何分工：RGB 模型判断“是什么”，深度/点云判断“在哪里、怎么抓”。
5. 系统必须有状态机、超时、重试、看门狗和日志。比赛失败往往来自系统集成，而不是单个算法指标。

## 3. 推荐主路线：FAST-LIO2 类 LiDAR-IMU SLAM + 2.5D 可通行性地图 + 前沿探索 + 导航控制

这是当前最贴合硬件和赛题的首选实现路线。

### 3.1 技术组成

```text
Mid-360 points + IMU
→ FAST-LIO2 类 LiDAR-Inertial Odometry
→ 局部/全局点云地图
→ elevation_mapping / grid_map 多层高程地图
→ traversability costmap
→ frontier-based exploration
→ Nav2 或自定义局部控制器
→ /chassis/cmd_vel TwistStamped 适配
```

成熟参考路径：

- FAST-LIO / FAST-LIO2：开源 LiDAR-IMU 紧耦合里程计与建图，适合 Mid-360 这类非传统扫描模式雷达。FAST-LIO2 使用直接点到地图配准、迭代 EKF 和 ikd-Tree 增量地图结构，目标是高频、低算力、鲁棒定位。
- elevation_mapping：ANYbotics 的机器人中心高程建图方案，面向崎岖地形局部导航，融合距离传感器观测和位姿不确定性。
- grid_map：二维网格多层数据结构，可承载 elevation、variance、surface normal、roughness、traversability 等层。
- Nav2：ROS 2 成熟导航框架，负责行为树、全局规划、局部控制、代价地图和恢复行为。

### 3.2 算法原理

LiDAR-IMU SLAM 先用 IMU 在两帧雷达之间预测短时间位姿，再用点云与局部地图配准校正漂移。典型流程：

1. IMU 递推预测姿态、速度、位置；
2. 利用预测轨迹对一帧点云去畸变；
3. 将当前点云配准到局部地图；
4. 用 EKF 或优化方法更新状态；
5. 将新点云插入局部/全局地图；
6. 输出 `odom -> base_link` 或 `map -> base_link`。

2.5D 可通行性地图不是简单占据栅格，而是多层 grid map：

- `elevation`：地面高度；
- `variance`：高度不确定性；
- `slope`：坡度；
- `roughness`：粗糙度；
- `step_height`：局部高程突变；
- `positive_obstacle`：凸起岩石；
- `negative_obstacle`：坑洼/缺测风险；
- `inflation`：按车体宽度膨胀；
- `traversability`：综合通行代价。

初始通行规则建议：

- `< 60 mm`：可跨越但提高代价并限速；
- `60–100 mm`：默认避让，除非实测底盘能稳定通过；
- `> 100 mm`：不可通行；
- `≤ 6°` 坡：可通行但限速；
- 未知区域：探索阶段可谨慎进入，返航阶段提高风险。

前沿探索将地图分为已知空闲、已知障碍、未知区域。已知空闲和未知区域的边界就是前沿。前沿选择不能只看信息增益，还要结合任务时间和返航风险：

```text
frontier_score =
  信息增益
  - 路径长度/耗时
  - 地形风险
  - 返航风险
  + 目标检测热度
```

## 4. 备选路线 A：LIO-SAM 类因子图 LiDAR-IMU SLAM

LIO-SAM 将 IMU、雷达里程计、回环等约束放入因子图。每个关键帧是位姿节点，传感器观测是因子，优化器通过最小化残差获得全局一致轨迹。

优点：

- 回环和全局一致性强；
- 因子图结构清楚，方便技术报告展示；
- 可扩展轮速、视觉、先验地图等约束。

风险：

- 对 IMU、外参和时间同步更敏感；
- Mid-360 适配和调参成本较高；
- 算力压力和工程复杂度高于 FAST-LIO2 类路线。

建议定位：作为第二阶段对照方案，不作为第一套必须跑通的主路线。

## 5. 备选路线 B：OctoMap/体素地图 + 3D 洞穴结构表达

OctoMap 用八叉树表达 3D 占据概率。传感器射线经过的体素更新为空闲，射线终点更新为占用。它适合表达洞壁、岩壁平台、障碍和未知空间。

对本项目的作用：

- 保存熔岩洞 3D 结构；
- 辅助目标搜索视域分析；
- 辅助区分地面障碍和岩壁/顶棚；
- 可投影为 2D/2.5D 导航代价地图。

风险：

- 直接用 3D 体素做地面导航成本高；
- 体素地图到可通行地图需要自定义投影规则；
- 许多 3D 探索框架来自 UAV 场景，不能直接照搬到地面车。

建议定位：作为主路线地图增强模块，而不是第一阶段导航主地图。

## 6. 目标识别、三维定位与抓取路线

导航只是基础，最终得分依赖目标采集。

推荐感知链路：

```text
Gemini RGB
→ YOLO11 / YOLOv8 检测或实例分割
→ ROI 内 SAM2 精细分割（可选，低频）
→ 深度图/深度点云提取目标点云
→ 地面/岩壁背景剔除
→ 点云聚类与离群点去除
→ PCA / OBB / 模型匹配
→ 输出目标中心、尺寸、朝向、抓取点
```

工程策略：

1. YOLO 负责实时发现候选目标；SAM2 只在候选 ROI 或抓取前低频运行。
2. 地面目标和岩壁目标分开处理。地面目标先做地面平面剔除；岩壁平台目标先做局部支撑面/背景分割。
3. 对 5 cm 小目标必须多帧滤波，单帧深度噪声会显著影响抓取点。
4. 对称物体不要强行输出稳定 yaw。方木块可降级为中心抓取，让夹爪容错。
5. 若初赛后拿到特殊形状 3D 模型，可增加模板匹配或模型关键点检测。

视觉伺服流程：

```text
SEARCH_TARGET
→ COARSE_YAW_ALIGN
→ APPROACH
→ FINE_LATERAL_ALIGN
→ FINE_YAW_ALIGN
→ ARM_PREGRASP
→ GRASP
→ VERIFY_GRASP
```

每个阶段必须有速度限幅、误差死区、连续多帧稳定判据、目标丢失停车、超时退出和失败重试。

## 7. 完整任务状态机

建议所有模块由状态机调度，不让导航、检测、机械臂各自抢控制权。

```text
INIT
→ SELF_CHECK
→ LEAVE_BASE
→ ENTER_CAVE
→ EXPLORE
→ TARGET_CANDIDATE_FOUND
→ APPROACH_TARGET
→ ALIGN_TARGET
→ GRASP_TARGET
→ VERIFY_GRASP
→ DECIDE_CONTINUE_OR_RETURN
→ RETURN_HOME
→ ALIGN_DROPOFF
→ UNLOAD
→ FINISH
```

异常状态：

```text
LOCALIZATION_DEGRADED
NAVIGATION_STUCK
TARGET_LOST
GRASP_FAILED
LOW_TIME_REMAINING
LOW_BATTERY
EMERGENCY_STOP
```

策略建议：

- 已抓到至少 1 个目标后，若剩余时间不足，优先返航；
- 未抓到目标但探索超时，返航争取出发/探索部分得分；
- 定位退化时降速或原地扫描，不继续高速前进；
- 导航卡死时后退、旋转、重规划，超过阈值换前沿或返航；
- 机械臂动作期间底盘锁止。

## 8. 分阶段实施计划

### 阶段 0：基础设施

目标：让所有算法都可验证。

- 统一 TF：`map -> odom -> base_link -> livox_frame / camera_frame / arm_base`；
- 建立 rosbag 录制模板；
- 建立仿真回归任务；
- 建立真值评估脚本；
- 监控传感器频率、控制频率、RTF、CPU/GPU/内存；
- 建立启动/停止脚本，避免残留进程污染测试。

验收：

- 启动后 30 s 所有关键话题稳定；
- TF 无断链；
- rosbag 可完整回放；
- 真值只被评估脚本读取。

### 阶段 1：LiDAR-IMU 里程计基线

- 接入 Mid-360 点云和 IMU；
- 检查时间同步；
- 完成 LiDAR-IMU 外参初值；
- 跑通 FAST-LIO2 类里程计；
- 输出 `odom -> base_link`；
- 与仿真真值离线对比。

验收：

- 原地旋转、直行、坡道进入无明显位姿跳变；
- 轨迹连续；
- 算法频率满足导航闭环；
- GUI 关闭后实时性明显提高。

### 阶段 2：2.5D 可通行性建图

- 点云体素滤波；
- 地面提取；
- 高程地图生成；
- 坡度、粗糙度、高程突变计算；
- 正障碍/负障碍风险估计；
- 输出 costmap 或 grid_map 多层地图。

验收：

- 10 cm 石块标为高风险；
- 6 cm 以下小障碍可配置为可跨越高代价；
- 6° 坡道可通行但限速；
- 未知坑洼不误判为安全空地。

### 阶段 3：未知环境探索导航

- 前沿检测；
- 前沿聚类和评分；
- 全局路径规划；
- 局部避障；
- `/chassis/cmd_vel` 适配；
- 记录已访问路径和返航关键帧。

验收：

- 能从基地进入洞口；
- 能沿通道探索并处理岔路；
- 能绕开不可通行障碍；
- 任意时刻可触发返航。

### 阶段 4：目标识别与三维定位

- 采集方木块和特殊形状物数据；
- 训练 YOLO 检测/分割模型；
- 接入 Gemini 深度；
- ROI 点云提取；
- 聚类、PCA/OBB、抓取点估计；
- 多帧滤波和置信度评分。

验收：

- 地面目标和 50 cm 以下岩壁目标都能检测；
- 误检岩石率可控；
- 目标中心误差满足机械臂抓取容差；
- 低光照下主动光源/曝光策略有效。

### 阶段 5：车体对准与机械臂抓取

- 车体视觉伺服；
- 机械臂正逆解或预设抓取轨迹；
- 夹爪闭合控制；
- 抓取成功确认；
- 失败重试。

验收：

- 静态目标抓取成功率先达标；
- 不同初始角度和距离下能进入机械臂工作空间；
- 抓取后不会空载误判为成功。

### 阶段 6：返航与卸载

- 保存 home 位姿和返航路径；
- 返航优先走已知安全区域；
- 基地和卸载区定位；
- 卸载机构控制；
- 目标落区确认。

验收：

- 已抓取目标后稳定返回基地；
- 卸载不掉出 `0.5 m × 0.5 m` 区域；
- 任务结束后车辆停稳在基地范围内。

### 阶段 7：全流程回归

每次完整测试记录：

- 是否离开基地；
- 探索距离；
- 建图覆盖率；
- 目标发现数量；
- 抓取成功数量；
- 成功带回数量；
- 卸载成功数量；
- 总耗时；
- 卡死次数；
- 碰撞次数；
- 定位退化次数；
- RTF / CPU / GPU / 内存；
- rosbag 和日志路径。

## 9. 仓库落地建议

建议按职责补齐这些包：

- `lunar_rover_localization`：LiDAR-IMU SLAM 接入、TF 输出、真值隔离；
- `lunar_rover_mapping`：点云滤波、高程地图、可通行性地图、costmap 输出；
- `lunar_rover_exploration`：前沿检测、前沿评分、探索目标生成、返航策略；
- `lunar_rover_perception`：YOLO 推理、SAM2 ROI 分割、深度点云目标定位、PCA/OBB；
- `lunar_rover_manipulation`：机械臂运动学、夹爪控制、抓取轨迹、抓取确认；
- `lunar_rover_mission`：任务状态机、心跳、超时、模块仲裁、比赛计时；
- `lunar_rover_eval`：仿真真值对比、任务指标统计、自动回归报告。

## 10. 仿真场景改造建议

当前 `reachable_maze` 适合底盘动作和可视化调试，但不足以代表比赛。建议新增 `lava_tube` 类场景：

- 不规则洞壁；
- 约 1 m 可通行宽度；
- 入口 6° 下坡；
- 沙地材质；
- 半埋低矮石块；
- 坑洼/负障碍；
- 低光照与洞口强光；
- 地面目标和岩壁平台目标；
- 基地 1.5 m × 1.5 m；
- 卸载区 0.5 m × 0.5 m；
- 多个随机种子用于回归。

仿真分两类：

- 调试场景：GUI + RViz，低速、透明墙、低点云显示负载；
- 回归场景：headless，固定随机种子，记录 rosbag 和指标。

## 11. 参考资料与技术来源

项目资料：

- `docs/project_info/README.md`
- `docs/project_info/XH-202605_月球熔岩洞机器人智能自主采样任务.pdf`
- `docs/project_info/260714挑战杯问题汇总与答疑 (1).pdf`
- `docs/project_info/场地实拍，比较重要.png`
- `docs/project_info/往届参考/IRMV月极巡拓者技术路线总结.md`

成熟开源与论文路径：

- FAST-LIO GitHub：https://github.com/hku-mars/fast_lio
- FAST-LIO2 paper：https://arxiv.org/abs/2107.06829
- LIO-SAM GitHub：https://github.com/TixiaoShan/LIO-SAM
- Nav2 官方文档：https://docs.nav2.org/
- Nav2 项目主页：https://nav2.org/
- OctoMap：https://octomap.github.io/
- ANYbotics elevation_mapping：https://github.com/anybotics/elevation_mapping
- ANYbotics grid_map：https://github.com/anybotics/grid_map
- FUEL 探索框架：https://github.com/HKUST-Aerial-Robotics/FUEL
- ETH ASL mav_active_3d_planning：https://github.com/ethz-asl/mav_active_3d_planning
- Ultralytics YOLO11 文档：https://docs.ultralytics.com/models/yolo11/
- SAM2 GitHub：https://github.com/facebookresearch/sam2
- PCL 欧式聚类教程：https://pcl.readthedocs.io/projects/tutorials/en/master/cluster_extraction.html

## 12. 结论

本项目最可靠的实现路线不是“2D 迷宫 SLAM”，而是：

```text
LiDAR-IMU 主定位
→ 2.5D 地形可通行性
→ 前沿探索
→ RGB-D 目标识别抓取
→ 状态机调度
→ 返航卸载
```

第一阶段目标应定义为：

```text
在熔岩洞仿真场景中，
小车从基地出发，
通过 Mid-360 + IMU 完成定位建图，
用 2.5D 地图避开不可通行障碍，
探索并发现至少一个目标，
完成接近、抓取、返航和卸载的最小闭环。
```

这个闭环跑通后，后续优化就有明确抓手：定位漂移、地图误判、探索效率、目标误检、抓取失败、返航稳定性和系统实时性都可以被单独量化和迭代。
