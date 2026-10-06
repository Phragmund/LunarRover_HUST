# Web 统一控制台手眼标定流程

手眼标定现在由统一控制台承载，不再创建 OpenCV 独立窗口，也不接收本地 `p`/`q` 按键。

## 使用步骤

1. 启动相机、动态手眼 TF 和统一 Web 控制台，打开相机页并连接 ROS。
2. 点击“启动自动标定”。控制台画面会切换为标定节点发布的标注流，标注包括 ChArUco 标记、角点、坐标轴、角点数量和当前样本数。
3. 保持整块 7×10 ChArUco 板可见，手工移动机械臂到新的姿态。机械臂停止后无需点击采样；条件满足时自动保存一组。
4. 点击“结束标定并保存”。节点会写入 `samples.jsonl`、标注图和 `session_manifest.json`，并打包为同目录的 ZIP。状态栏出现“下载标定归档”后即可下载。

## 自动采样条件

默认参数如下，均可通过 ROS 参数覆盖：

- ChArUco 角点不少于 35 个；
- 关节反馈连续稳定约 0.8 s，关节变化不超过 0.6°；
- 相机到标定板平移变化不超过 4 mm、旋转变化不超过 1.5°，连续稳定约 0.8 s；
- 两次采样至少间隔 1.5 s，且相对上一组姿态至少变化 2°；
- 缺少相机帧、相机内参、关节反馈或姿态不满足条件时只更新状态，不写入样本。

控制台状态栏会显示角点/标记数量、关节和板位稳定性，以及当前未采样原因。

## ROS 接口

| 接口 | 作用 |
|---|---|
| `/camera/color/image_raw` | 输入 RGB 图像（BEST_EFFORT） |
| `/camera/color/camera_info` | 输入内参 |
| `/joint_states` | 输入机械臂反馈 |
| `/handeye/calibration_overlay/compressed` | 输出带标注 JPEG，统一控制台转发为 `handeye_camera` |
| `/handeye/calibration_status` | 输出 JSON 状态、样本数、稳定性、归档路径 |
| `/handeye/calibration_command` | 仅保留 `q` 结束命令；启动由 Web 控制台管理进程 |
| `GET /handeye/download` | 下载最近一次已结束的 ZIP 归档 |

相机标定输入采用 BEST_EFFORT，避免与 Gemini 传感器数据 QoS 不兼容。输出标注流同样采用 BEST_EFFORT。

## 归档与复现

每次启动都创建独立时间戳目录，不覆盖旧数据。ZIP 内含原始样本 JSONL、每组标注图和自动采样参数清单。完成采样后，再使用 [`tools/solve_handeye_samples.py`](../tools/solve_handeye_samples.py) 做筛选、求解和残差检查；求解结果要登记到 [`docs/tf_calibration_history.md`](tf_calibration_history.md) 和版本注册表，不能直接用 WebUI 样本覆盖在线 TF。

相机硬件没有 RGB 帧时，WebUI 会保持“等待相机画面”，不会伪造样本，也不会移动机械臂。
