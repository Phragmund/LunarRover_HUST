#!/usr/bin/env python3
"""Static checks for the lunar rover simulation, mapping, and navigation workspace."""

import json
import py_compile
import re
import stat
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SKIP_PARTS = {"build", "install", "log", "third_party", ".deps"}
errors = []


def tracked_files(pattern):
    for path in ROOT.rglob(pattern):
        if not any(part in SKIP_PARTS for part in path.parts):
            yield path


def check_xml():
    for pattern in ("package.xml", "*.sdf", "*.urdf", "*.xacro", "*.world"):
        for path in tracked_files(pattern):
            try:
                ET.parse(path)
            except ET.ParseError as error:
                errors.append(f"XML {path.relative_to(ROOT)}: {error}")


def check_yaml_json():
    for path in tracked_files("*.yaml"):
        try:
            yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception as error:  # noqa: BLE001
            errors.append(f"YAML {path.relative_to(ROOT)}: {error}")
    for path in tracked_files("*.json"):
        try:
            json.loads(path.read_text(encoding="utf-8"))
        except Exception as error:  # noqa: BLE001
            errors.append(f"JSON {path.relative_to(ROOT)}: {error}")


def check_python():
    with tempfile.TemporaryDirectory(prefix="lunar_rover_pycheck_") as temp_dir:
        for index, path in enumerate(tracked_files("*.py")):
            try:
                py_compile.compile(
                    str(path),
                    cfile=str(Path(temp_dir) / f"{index}.pyc"),
                    doraise=True,
                )
            except py_compile.PyCompileError as error:
                errors.append(f"Python {path.relative_to(ROOT)}: {error.msg}")


def check_packages():
    names = set()
    for manifest in tracked_files("package.xml"):
        try:
            name = ET.parse(manifest).getroot().findtext("name")
        except ET.ParseError:
            continue
        if name:
            names.add(name)

    required = {
        "lunar_rover_msgs",
        "lunar_rover_description",
        "lunar_rover_simulation",
        "lunar_rover_control",
        "lunar_rover_sensors",
        "lunar_rover_bringup",
        "lunar_rover_localization",
        "lunar_rover_exploration",
    }
    missing = sorted(required - names)
    if missing:
        errors.append("Missing base packages: " + ", ".join(missing))

    removed = {
        "lunar_rover_perception",
        "lunar_rover_missions",
    }
    present = sorted(removed & names)
    if present:
        errors.append("Algorithm packages remain: " + ", ".join(present))


def check_contracts():
    state = ROOT / "src/lunar_rover_msgs/msg/ChassisState.msg"
    if not state.exists():
        errors.append("ChassisState.msg is missing")

    for script in tracked_files("*.sh"):
        if not script.stat().st_mode & stat.S_IXUSR:
            errors.append(f"Script is not executable: {script.relative_to(ROOT)}")

    for path in tracked_files("*.sdf"):
        if not path.match("*/lunar_rover_simulation/worlds/*.sdf"):
            continue
        text = path.read_text(encoding="utf-8")
        match = re.search(r"<gravity>\s*([^<]+)\s*</gravity>", text)
        if not match or match.group(1).strip() != "0 0 -9.80665":
            errors.append(
                f"World gravity is not standard Earth gravity: {path.relative_to(ROOT)}"
            )
        physics = ET.parse(path).getroot().find(".//physics")
        if physics is None:
            errors.append(f"World physics block is missing: {path.relative_to(ROOT)}")
            continue
        expected_physics = {
            "max_step_size": "0.001",
            "real_time_factor": "1.0",
            "real_time_update_rate": "1000",
        }
        for tag, expected in expected_physics.items():
            value = physics.findtext(tag)
            if value is None or value.strip() != expected:
                errors.append(
                    f"World physics {tag} must be {expected}: {path.relative_to(ROOT)}"
                )

    check_controller_config(ROOT / "src/lunar_rover_control/config/controllers.yaml")
    check_controller_config(ROOT / "src/lunar_rover_simulation/config/controllers.yaml")
    check_simulation_closed_loop_launch()

    source_root = ROOT / "src"
    source_text = "\n".join(
        path.read_text(encoding="utf-8", errors="ignore")
        for pattern in ("*.py", "*.cpp", "*.hpp", "*.yaml", "package.xml")
        for path in source_root.rglob(pattern)
        if not any(part in SKIP_PARTS for part in path.parts)
    ).lower()
    forbidden = (
        "rtabmap",
        "icp_odometry",
        "terrain_perception",
        "wheel_odometry",
    )
    for token in forbidden:
        if token in source_text:
            errors.append(f"Removed algorithm reference remains: {token}")


def check_controller_config(path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    relative = path.relative_to(ROOT)
    manager = data.get("controller_manager", {}).get("ros__parameters", {})
    if manager.get("update_rate") != 100:
        errors.append(f"Controller manager update_rate must be 100 Hz: {relative}")

    expected_types = {
        "joint_state_broadcaster": "joint_state_broadcaster/JointStateBroadcaster",
        "rover_controller": "lunar_rover_control/RoverController",
        "rocker_differential_controller":
            "lunar_rover_control/RockerDifferentialController",
    }
    for name, expected_type in expected_types.items():
        configured = manager.get(name, {}).get("type")
        if configured != expected_type:
            errors.append(
                f"{relative} must declare {name} as {expected_type}"
            )

    rover = data.get("rover_controller", {}).get("ros__parameters", {})
    rocker = data.get("rocker_differential_controller", {}).get("ros__parameters", {})
    if rover.get("command_topic") != "/chassis/cmd_vel":
        errors.append(f"{relative} rover_controller must subscribe to /chassis/cmd_vel")
    if rocker.get("left_rocker_joint") != "left_rocker_joint":
        errors.append(f"{relative} rocker controller left joint mismatch")
    if rocker.get("right_rocker_joint") != "right_rocker_joint":
        errors.append(f"{relative} rocker controller right joint mismatch")


def check_simulation_closed_loop_launch():
    launch_path = ROOT / "src/lunar_rover_simulation/launch/simulation.launch.py"
    text = launch_path.read_text(encoding="utf-8")
    relative = launch_path.relative_to(ROOT)
    required_tokens = (
        "joint_state_broadcaster",
        "rocker_differential_controller",
        "rover_controller",
        "sim_chassis_state_node",
        "OnProcessExit(target_action=spawn_rover",
        "--param-file",
        "controller_config",
    )
    for token in required_tokens:
        if token not in text:
            errors.append(f"Closed-loop simulation launch missing {token}: {relative}")
    if 'DeclareLaunchArgument("spawn_x", default_value="-3.92")' not in text:
        errors.append(
            f"Default spawn_x must stay at the reachable-maze start cell center: {relative}"
        )
    if 'DeclareLaunchArgument("spawn_y", default_value="0.0")' not in text:
        errors.append(
            f"Default spawn_y must stay at the reachable-maze start cell center: {relative}"
        )


def main():
    check_xml()
    check_yaml_json()
    check_python()
    check_packages()
    check_contracts()
    if errors:
        print("Static validation failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1
    package_count = sum(1 for _ in tracked_files("package.xml"))
    print(f"Static validation passed for {package_count} base ROS packages.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
