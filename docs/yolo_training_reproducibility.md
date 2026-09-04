# 本地 YOLO 检测训练与权重验收

目标检测默认使用 Ultralytics `detect` 任务，而不是必须依赖实例分割。数据集目录不提交
Git；其最小布局为：

```text
datasets/lunar_targets/
  images/{train,val,test}/<image>.jpg
  labels/{train,val,test}/<image>.txt
```

标签采用 YOLO 框格式：`class x_center y_center width height`，坐标归一化到 `[0, 1]`。
当前部署权重的类别顺序为 `wood_block`、`artificial_object`。`target_dataset.yaml`
保留 `rock_sample` 作为后续专项训练的模板类别；未重新训练并验收包含该类别的权重前，
不得把它加入默认部署类别白名单。

## 环境与训练

```bash
scripts/setup_target_perception.sh
.deps/target_perception_venv/bin/python scripts/train_target_detector.py \
  --task detect --weights yolo11n.pt --data src/lunar_rover_manipulation/config/target_dataset.yaml
```

分割数据集可使用 `--task segment --weights yolo11n-seg.pt`。训练产物默认保存在被 Git
忽略的 `.deps/target_segmentation_runs/`；必须保留该目录下的 `args.yaml`、`results.csv`、
`weights/best.pt` 与数据版本说明，才能审计“自训练”结论。

## 部署前验收

```bash
.deps/target_perception_venv/bin/python scripts/inspect_yolo_checkpoint.py \
  /absolute/path/best.pt --expected-task detect \
  --expected-class wood_block --expected-class artificial_object
```

当前已验收的 `weights/best.pt` 覆盖 `wood_block` 与 `artificial_object`；`rock_sample`
保留在统一数据集类别表中，待专项训练权重交付后再纳入部署验收。实机主入口默认从
`weights/best.pt` 加载此权重；仅在替换权重时设置
`TARGET_DETECTOR_MODEL=/absolute/path/best.pt`。旧变量 `TARGET_SEGMENTATION_MODEL`
仍兼容，但新配置请使用前者。

本次初赛审验将 `weights/best.pt` 随源码提交；其 SHA-256、类别和加载方式见
[`weights/README.md`](../weights/README.md)。训练图片与标签受数据来源限制不随仓库发布，
但数据格式、训练命令、依赖安装脚本及验收命令均已提供。

检测模型没有像素级 mask 时，`target_detector_node.py` 会发布检测框的矩形 mask，以维持
RGB-D 定位；分割模型会自动发布其精确 mask。
