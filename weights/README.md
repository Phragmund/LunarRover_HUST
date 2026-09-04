# 初赛审验 YOLO 权重

`best.pt` 是实机默认加载的自训练 Ultralytics `detect` 权重；实机主启动脚本和 launch
默认使用该文件，不需要额外下载模型。

| 项目 | 值 |
| --- | --- |
| 任务 | `detect` |
| 部署类别 | `wood_block`、`artificial_object` |
| 文件 | `weights/best.pt` |
| SHA-256 | `92e713898ea0f360f8b45044a1255067f9f7706f2666b3ed3a28c09fb2d5e40a` |

验收命令：

```bash
scripts/setup_target_perception.sh
.deps/target_perception_venv/bin/python scripts/inspect_yolo_checkpoint.py \
  weights/best.pt --expected-task detect \
  --expected-class wood_block --expected-class artificial_object
```

训练数据受数据来源与体积限制不随仓库发布；可复现训练的目录规范、数据 YAML、训练入口和
依赖版本清单位于 `docs/yolo_training_reproducibility.md`、
`src/lunar_rover_manipulation/config/target_dataset.yaml`、`scripts/train_target_detector.py`
与 `dependencies/target_perception.txt`。
