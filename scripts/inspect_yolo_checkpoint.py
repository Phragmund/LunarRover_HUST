#!/usr/bin/env python3
"""Print auditable metadata for an Ultralytics checkpoint before deployment."""

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("weights", type=Path)
    parser.add_argument("--expected-task", choices=("detect", "segment"))
    parser.add_argument("--expected-class", action="append", default=[])
    args = parser.parse_args()
    if not args.weights.is_file():
        raise SystemExit(f"weights not found: {args.weights}")

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit("Ultralytics is not installed; run setup_target_perception.sh") from exc

    model = YOLO(str(args.weights))
    names = {int(key): str(value) for key, value in model.names.items()}
    print(f"weights: {args.weights.resolve()}")
    print(f"task: {model.task}")
    print("classes:")
    for index, name in names.items():
        print(f"  {index}: {name}")
    if args.expected_task and model.task != args.expected_task:
        raise SystemExit(
            f"unexpected task {model.task!r}; expected {args.expected_task!r}"
        )
    missing = sorted(set(args.expected_class) - set(names.values()))
    if missing:
        raise SystemExit(f"checkpoint is missing expected classes: {', '.join(missing)}")


if __name__ == "__main__":
    main()
