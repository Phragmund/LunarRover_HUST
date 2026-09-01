# lunar_rover_description

机器人描述包，提供六轮摇臂月球车模型和模型可视化配置。

## 主要文件

| 路径 | 说明 |
| --- | --- |
| `urdf/lunar_rover.urdf.xacro` | 当前主模型，包含车体、六轮、四个转向关节、摇臂/副摇臂、传感器外参、五自由度机械臂和仿真控制接口。 |
| `urdf/lunar_rover_original.urdf.xacro` | 历史模型备份，用于对比。 |
| `launch/display.launch.py` | 不启动 Gazebo，仅用 `robot_state_publisher`、`joint_state_publisher_gui` 和 RViz 查看模型。 |
| `rviz/lunar_rover.rviz` | 模型和原始传感器查看配置。 |
| `rviz/lunar_rover_depth_points.rviz` | 深度点云观察配置。 |

## 使用

```bash
ros2 launch lunar_rover_description display.launch.py rviz:=true
```

修改模型时需要同步检查传感器 frame、控制关节名和下位机接口文档，避免破坏 `lunar_rover_control` 与实车 `/joint_states` 契约。

## 机械臂（Rover_0）

机械臂基座位于 Gemini335 深度相机正后方偏上的车体顶板处（`base_link` 系下约 `(0.13, 0, 0.056)` m）：

| 关节 | 类型 | 轴 | 说明 |
| --- | --- | --- | --- |
| `arm_joint1` | revolute | z | J1 偏航，基座处；J1=0 为折叠姿态，J1=-π/2 时前伸 |
| `arm_joint2` | revolute | y | J2 肩俯仰，位于 J1 正上方 0.07545 m |
| `arm_joint3` | revolute | y | J3 肘俯仰 |
| `arm_joint4` | revolute | y | J4 腕俯仰 |
| `arm_joint5` | prismatic | y | J5 夹爪开合 |

臂段尺寸：J2→J3 = 0.11391 m，J3→J4 = 0.11000 m，J4→夹爪工作中心 = 0.15601 m。

零位为折叠姿态：机械臂贴放在车体左侧（x=0.13，y 0~0.108，z 0.066~0.181），位于 Gemini335 镜头后方和 Mid-360 视场下方，不影响传感器工作。机械臂总质量约 0.3 kg，远小于整车质量。

机械臂各 link 设置了 `<gazebo reference="arm_link*"><selfCollide>true</selfCollide></gazebo>`，禁用与同模型其他 link 的碰撞，避免相邻关节几何重叠和折叠零位接触导致的 Gazebo 抖动；夹爪手指仍可与外部物块模型正常碰撞。
