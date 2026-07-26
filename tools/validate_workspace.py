#!/usr/bin/env python3
"""Static checks that do not require a sourced ROS installation."""

import json
import py_compile
import re
import stat
import sys
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
    patterns = ("package.xml", "*.sdf", "*.urdf", "*.xacro", "*.world")
    for pattern in patterns:
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
    for path in tracked_files("*.py"):
        try:
            py_compile.compile(str(path), doraise=True)
        except py_compile.PyCompileError as error:
            errors.append(f"Python {path.relative_to(ROOT)}: {error.msg}")


def check_packages():
    names = {}
    for manifest in tracked_files("package.xml"):
        try:
            name = ET.parse(manifest).getroot().findtext("name")
        except ET.ParseError:
            continue
        if not name:
            errors.append(f"{manifest.relative_to(ROOT)} has no package name")
        elif name in names:
            errors.append(
                f"Duplicate package {name}: {manifest.relative_to(ROOT)} and "
                f"{names[name].relative_to(ROOT)}"
            )
        else:
            names[name] = manifest

    required = {
        "lunar_rover_msgs",
        "lunar_rover_description",
        "lunar_rover_simulation",
        "lunar_rover_control",
        "lunar_rover_sensors",
        "lunar_rover_localization",
        "lunar_rover_perception",
        "lunar_rover_navigation",
        "lunar_rover_missions",
        "lunar_rover_bringup",
    }
    missing = sorted(required - names.keys())
    if missing:
        errors.append("Missing packages: " + ", ".join(missing))


def check_contracts():
    state = ROOT / "src/lunar_rover_msgs/msg/ChassisState.msg"
    required_lines = {
        "uint8 mode",
        "uint32 fault_bits",
        "bool estop",
        "bool enabled",
        "bool watchdog_ok",
    }
    if state.exists():
        contents = set(state.read_text(encoding="utf-8").splitlines())
        missing = required_lines - contents
        if missing:
            errors.append("ChassisState missing: " + ", ".join(sorted(missing)))
    else:
        errors.append("ChassisState.msg is missing")

    for script in tracked_files("*.sh"):
        mode = script.stat().st_mode
        if not mode & stat.S_IXUSR:
            errors.append(f"Script is not executable: {script.relative_to(ROOT)}")

    forbidden = []
    for path in list(tracked_files("*.sdf")) + list(tracked_files("*.world")):
        text = path.read_text(encoding="utf-8")
        if "-1.62" in text:
            forbidden.append(str(path.relative_to(ROOT)))
        if path.match("*/lunar_rover_simulation/worlds/*.sdf"):
            match = re.search(r"<gravity>\s*([^<]+)\s*</gravity>", text)
            if not match:
                errors.append(f"World has no gravity: {path.relative_to(ROOT)}")
            elif match.group(1).strip() != "0 0 -9.80665":
                errors.append(
                    f"World gravity is not standard Earth gravity: {path.relative_to(ROOT)}"
                )
    if forbidden:
        errors.append("Moon gravity remains in: " + ", ".join(forbidden))

    perception_config = ROOT / "src/lunar_rover_perception/config/terrain_perception.yaml"
    nav_config = ROOT / "src/lunar_rover_navigation/config/nav2_params.yaml"
    safety_config = ROOT / "src/lunar_rover_bringup/config/safety.yaml"
    if perception_config.exists() and nav_config.exists() and safety_config.exists():
        perception_text = perception_config.read_text(encoding="utf-8")
        nav_text = nav_config.read_text(encoding="utf-8")
        safety_text = safety_config.read_text(encoding="utf-8")
        if "/terrain/traversability" not in perception_text:
            errors.append("Perception config does not publish /terrain/traversability")
        if "/terrain/traversability" not in nav_text:
            errors.append("Nav2 config does not consume /terrain/traversability")
        if "/terrain/traversability" not in safety_text:
            errors.append("Safety config does not monitor /terrain/traversability")

    bringup_launch = ROOT / "src/lunar_rover_bringup/launch/bringup.launch.py"
    if bringup_launch.exists():
        text = bringup_launch.read_text(encoding="utf-8")
        if "wheel_odometry_node" not in text:
            errors.append("Real bringup does not start wheel_odometry_node")
        if "controller_manager" in text:
            errors.append("Top-level bringup must not start controller_manager in real mode")

    real_gemini = ROOT / "src/lunar_rover_sensors/config/gemini335_real.yaml"
    if real_gemini.exists():
        text = real_gemini.read_text(encoding="utf-8")
        unsupported = ("color_optical_frame_id", "depth_optical_frame_id")
        for key in unsupported:
            if key in text:
                errors.append(f"Orbbec config contains unsupported launch key: {key}")


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
    print(f"Static validation passed for {package_count} ROS packages.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
