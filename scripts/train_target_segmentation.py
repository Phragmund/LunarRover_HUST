#!/usr/bin/env python3
"""Fine-tune and export a small-target Ultralytics segmentation model."""

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
    parser.add_argument("--weights", default="yolo11n-seg.pt")
    parser.add_argument("--epochs", type=int, default=150)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--device", default="0")
    parser.add_argument(
        "--project", type=Path, default=repo_root / ".deps/target_segmentation_runs"
    )
    parser.add_argument("--name", default="yolo11n_lunar_targets")
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
    result = model.train(
        data=str(args.data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=str(args.project),
        name=args.name,
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
