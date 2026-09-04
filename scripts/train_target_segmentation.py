#!/usr/bin/env python3
"""Train a reproducible Ultralytics detector or segmenter for lunar targets.

The deployed target classes are bounding-box labels by default.  Segmentation
is optional: select it only when the dataset contains polygon labels and the
runtime needs an instance mask instead of the conservative box mask fallback.
"""

import argparse
from pathlib import Path


def parse_args():
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data",
        type=Path,
        default=repo_root
        / "src/lunar_rover_manipulation/config/target_dataset.yaml",
    )
    parser.add_argument(
        "--weights",
        default="yolo11n.pt",
        help="Ultralytics pretrained checkpoint or a prior local best.pt",
    )
    parser.add_argument(
        "--task", choices=("detect", "segment"), default="detect",
        help="Use detect for the repository's default box-label dataset.",
    )
    parser.add_argument("--epochs", type=int, default=150)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--device", default="0")
    parser.add_argument(
        "--project", type=Path, default=repo_root / ".deps/target_segmentation_runs"
    )
    parser.add_argument("--name", default="yolo11n_lunar_targets_detect")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Validate dataset layout and checkpoint task without training.",
    )
    parser.add_argument("--export-onnx", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.data.is_file():
        raise SystemExit(f"dataset configuration not found: {args.data}")
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            "Ultralytics is not installed; run scripts/setup_target_perception.sh"
        ) from exc

    model = YOLO(args.weights)
    if model.task != args.task:
        raise SystemExit(
            f"checkpoint task is {model.task!r}, but --task is {args.task!r}; "
            "choose a matching checkpoint/task pair"
        )
    print(f"training task: {args.task}")
    print(f"checkpoint: {args.weights}")
    print(f"classes before training: {model.names}")
    if args.dry_run:
        # Ultralytics performs the same dataset parsing and label validation as
        # training before epoch zero, without creating a training run.
        model.val(data=str(args.data), imgsz=args.imgsz, batch=args.batch, device=args.device)
        return
    result = model.train(
        data=str(args.data),
        task=args.task,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=str(args.project),
        name=args.name if args.task == "detect" else f"{args.name}_seg",
        patience=30,
        workers=4,
        close_mosaic=15,
        degrees=12.0,
        translate=0.15,
        scale=0.55,
        perspective=0.0005,
        hsv_h=0.03,
        hsv_s=0.45,
        hsv_v=0.35,
        fliplr=0.5,
        cache=False,
    )
    best = Path(result.save_dir) / "weights" / "best.pt"
    print(f"best weights: {best}")
    if args.export_onnx:
        exported = YOLO(str(best)).export(format="onnx", imgsz=args.imgsz, dynamic=True)
        print(f"exported model: {exported}")


if __name__ == "__main__":
    main()
