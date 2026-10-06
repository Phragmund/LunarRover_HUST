#!/usr/bin/env python3
"""Validate, list and persistently select versioned hand-eye calibrations."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tempfile

import numpy as np


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "src/lunar_rover_manipulation/config/handeye_calibrations.json"


def quaternion_to_matrix(values):
    q = np.asarray(values, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("quaternion must have four finite xyzw components")
    if not np.isclose(np.linalg.norm(q), 1.0, atol=1e-6, rtol=0):
        raise ValueError("quaternion must be normalized")
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def validate_transform(value, parent, child):
    if value.get("parent") != parent or value.get("child") != child:
        raise ValueError(f"expected transform {parent} -> {child}")
    translation = np.asarray(value["translation_xyz_m"], dtype=float)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("translation must have three finite components")
    rotation = quaternion_to_matrix(value["quaternion_xyzw"])
    if "matrix_4x4" in value:
        matrix = np.asarray(value["matrix_4x4"], dtype=float)
        expected = np.eye(4)
        expected[:3, :3], expected[:3, 3] = rotation, translation
        if matrix.shape != (4, 4) or not np.allclose(matrix, expected, atol=1e-8, rtol=0):
            raise ValueError("matrix, translation and quaternion disagree")


def load_registry(path=DEFAULT_CONFIG, calibration_id=""):
    registry = json.loads(Path(path).read_text(encoding="utf-8"))
    if registry.get("schema_version") != 1:
        raise ValueError("unsupported hand-eye registry schema")
    selected_id = calibration_id or registry["active_calibration"]
    if selected_id not in registry["profiles"]:
        raise ValueError(f"unknown calibration profile: {selected_id}")
    if registry["active_calibration"] not in registry["profiles"]:
        raise ValueError("active calibration is not in the registry")
    validate_transform(registry["base_T_arm"], "base_link", "arm_base_link")
    for profile in registry["profiles"].values():
        validate_transform(profile["tool_T_camera"], "handeye_tool_frame", "camera_color_optical_frame")
        validate_transform(profile["base_T_board_display"], "base_link", "charuco_board_frame")
    expected_aliases = [
        {"parent": "camera_color_optical_frame", "child": "gemini335_color_optical_frame"},
        {"parent": "camera_color_optical_frame", "child": "camera_depth_optical_frame"},
        {"parent": "camera_depth_optical_frame", "child": "gemini335_depth_optical_frame"},
    ]
    if registry["camera_aliases"] != expected_aliases:
        raise ValueError("camera aliases must preserve the single-owner, acyclic TF tree")
    geometry = registry["kinematics"]
    for key in ("j1_j2_axis_offset_m", "upper_arm_m", "forearm_m", "tool_m"):
        if not np.isfinite(geometry[key]) or geometry[key] <= 0:
            raise ValueError(f"invalid FK geometry: {key}")
    for key in ("zero_degrees", "ros_directions"):
        values = np.asarray(geometry[key], dtype=float)
        if values.shape != (5,) or not np.isfinite(values).all():
            raise ValueError(f"invalid joint mapping: {key}")
    if not all(direction in (-1, 1) for direction in geometry["ros_directions"]):
        raise ValueError("ROS joint directions must be +1 or -1")
    return registry, selected_id, registry["profiles"][selected_id]


def activate_profile(path, selected_id):
    path = Path(path)
    registry, _, _ = load_registry(path, selected_id)
    registry["active_calibration"] = selected_id
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".handeye-", delete=False) as handle:
            temp_path = Path(handle.name)
            handle.write(json.dumps(registry, ensure_ascii=False, indent=2) + "\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temp_path, path.stat().st_mode & 0o777)
        os.replace(temp_path, path)
    finally:
        if temp_path is not None and temp_path.exists():
            temp_path.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("list", "activate"))
    parser.add_argument("calibration_id", nargs="?")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    if args.action == "activate":
        if not args.calibration_id:
            parser.error("activate requires a calibration_id")
        activate_profile(args.config, args.calibration_id)
        print(f"Selected {args.calibration_id}; restart handeye TF to apply")
    else:
        registry, selected_id, _ = load_registry(args.config)
        for profile_id, profile in registry["profiles"].items():
            prefix = "*" if profile_id == selected_id else " "
            print(f"{prefix} {profile_id}: {profile['sample_count']} samples, {profile['method']}")


if __name__ == "__main__":
    main()
