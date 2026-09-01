#!/usr/bin/env python3
"""Solve eye-in-hand ChArUco samples using the real-arm FK convention."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


ZERO = np.array([90.0, 180.0, 135.0, 135.0, 0.0])
DIRECTION = np.array([1.0, -1.0, 1.0, 1.0, 1.0])
BASE_TO_SHOULDER = 0.07545
UPPER_ARM = 0.11391
FOREARM = 0.11000
TOOL = 0.15601


def make_transform(rotation, translation):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def fk_tool(positions):
    joints = ZERO[:4] + DIRECTION[:4] * np.degrees(np.asarray(positions[:4], dtype=float))
    j1, j2, j3, j4 = joints
    yaw = math.radians(j1 - 90.0)
    th2 = math.radians(j2)
    th3 = math.radians(j2 + (135.0 - j3))
    th4 = math.radians(j2 + (135.0 - j3) + (135.0 - j4))
    radius = BASE_TO_SHOULDER + UPPER_ARM * math.sin(th2)
    radius += FOREARM * math.sin(th3) + TOOL * math.sin(th4)
    z = -UPPER_ARM * math.cos(th2) - FOREARM * math.cos(th3)
    z -= TOOL * math.cos(th4)
    translation = np.array([radius * math.cos(yaw), radius * math.sin(yaw), z])
    x_axis = np.array(
        [math.cos(yaw) * math.sin(th4), math.sin(yaw) * math.sin(th4), -math.cos(th4)]
    )
    y_axis = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= np.linalg.norm(z_axis)
    rotation = np.column_stack([x_axis / np.linalg.norm(x_axis), y_axis, z_axis])
    return make_transform(rotation, translation)


def read_samples(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def solve(rows):
    gripper_to_base = [fk_tool(row["joint_state"]["positions"]) for row in rows]
    target_to_camera = []
    for row in rows:
        pose = row["camera_T_board"]
        rotation, _ = cv2.Rodrigues(np.asarray(pose["rotation_vector"], dtype=float))
        target_to_camera.append(make_transform(rotation, pose["translation_xyz_m"]))
    method = cv2.CALIB_HAND_EYE_PARK
    rotation, translation = cv2.calibrateHandEye(
        [item[:3, :3] for item in gripper_to_base],
        [item[:3, 3] for item in gripper_to_base],
        [item[:3, :3] for item in target_to_camera],
        [item[:3, 3] for item in target_to_camera],
        method=method,
    )
    return make_transform(rotation, np.asarray(translation).reshape(3))


def board_poses(rows, tool_to_camera):
    return [fk_tool(row["joint_state"]["positions"]) @ tool_to_camera @ target
            for row, target in zip(rows, target_camera_poses(rows))]


def target_camera_poses(rows):
    poses = []
    for row in rows:
        pose = row["camera_T_board"]
        rotation, _ = cv2.Rodrigues(np.asarray(pose["rotation_vector"], dtype=float))
        poses.append(make_transform(rotation, pose["translation_xyz_m"]))
    return poses


def rpy_deg(rotation):
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def solution_report(label, rows, tool_to_camera):
    poses = board_poses(rows, tool_to_camera)
    mean_translation = np.mean([pose[:3, 3] for pose in poses], axis=0)
    residuals = [float(np.linalg.norm(pose[:3, 3] - mean_translation) * 1000.0) for pose in poses]
    return {
        "label": label,
        "method": "PARK",
        "sample_indices": [row["index"] for row in rows],
        "sample_count": len(rows),
        "tool_T_camera": {
            "translation_xyz_m": tool_to_camera[:3, 3].tolist(),
            "rpy_deg_xyz": rpy_deg(tool_to_camera[:3, :3]),
            "matrix_4x4": tool_to_camera.tolist(),
        },
        "base_T_board_mean_translation_m": mean_translation.tolist(),
        "translation_residual_mm": {
            "mean": float(np.mean(residuals)),
            "median": float(np.median(residuals)),
            "max": float(np.max(residuals)),
        },
        "worst_samples": [
            {
                "index": rows[index]["index"],
                "translation_residual_mm": residuals[index],
                "charuco_corners": rows[index]["charuco_corner_count"],
                "image_joint_dt_ms": abs(
                    rows[index]["image_stamp"] - rows[index]["joint_state"]["stamp"]
                ) * 1000.0,
            }
            for index in np.argsort(residuals)[::-1][:6]
        ],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    parser.add_argument("--min-corners", type=int, default=35)
    parser.add_argument("--max-dt-ms", type=float, default=100.0)
    parser.add_argument("--max-residual-mm", type=float, default=40.0)
    args = parser.parse_args()
    rows = read_samples(args.session / "samples.jsonl")
    filtered = [
        row for row in rows
        if row["charuco_corner_count"] >= args.min_corners
        and abs(row["image_stamp"] - row["joint_state"]["stamp"]) * 1000.0 <= args.max_dt_ms
    ]
    initial = solve(filtered)
    initial_poses = board_poses(filtered, initial)
    mean_translation = np.mean([pose[:3, 3] for pose in initial_poses], axis=0)
    robust = [
        row for row, pose in zip(filtered, initial_poses)
        if np.linalg.norm(pose[:3, 3] - mean_translation) * 1000.0 <= args.max_residual_mm
    ]
    report = {
        "source_dir": str(args.session),
        "notes": [
            "FK uses corrected arm lengths: 75.45, 113.91, 110.00, 156.01 mm.",
            "tool_T_camera means camera_color_optical_frame expressed in the IK tool frame.",
            "Filter: image/joint dt <= %.1f ms, ChArUco corners >= %d, residual <= %.1f mm."
            % (args.max_dt_ms, args.min_corners, args.max_residual_mm),
        ],
        "data_quality": {
            "total_samples": len(rows),
            "filtered_samples": len(filtered),
            "robust_samples": len(robust),
            "charuco_corners_min_median_max": [
                min(row["charuco_corner_count"] for row in rows),
                float(np.median([row["charuco_corner_count"] for row in rows])),
                max(row["charuco_corner_count"] for row in rows),
            ],
            "image_joint_dt_ms_max": max(
                abs(row["image_stamp"] - row["joint_state"]["stamp"]) * 1000.0 for row in rows
            ),
            "excluded_by_initial_filter": [row["index"] for row in rows if row not in filtered],
            "excluded_by_residual_trim": [row["index"] for row in filtered if row not in robust],
        },
        "solutions": [
            solution_report("filtered", filtered, initial),
            solution_report("filtered_plus_residual_trim", robust, solve(robust)),
        ],
    }
    output = args.session / "handeye_initial_solution.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
