"""Regression checks for calibration provenance, rollback and TF ownership."""

import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import xacro

TOOLS = Path(__file__).resolve().parents[1]
ROOT = TOOLS.parent
sys.path.insert(0, str(TOOLS))

from handeye_calibration_registry import activate_profile, load_registry, quaternion_to_matrix
from handeye_tf_preview_node import HandeyeTfPreview
from solve_handeye_samples import fk_tool, read_samples, solve


class HandeyeCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.registry, self.active_id, self.active = load_registry()
        self.new_profile = self.registry["profiles"]["20260930_all12_park"]
        self.rows = read_samples(ROOT / "docs/calibration/handeye_samples/20260930_131448.jsonl")

    def test_all_archived_profiles_are_valid(self):
        self.assertGreaterEqual(len(self.registry["profiles"]), 6)
        self.assertIn(self.active_id, self.registry["profiles"])

    def test_new_calibrations_reproduce_from_archived_samples(self):
        raw = ROOT / self.new_profile["source_samples_archive"]
        self.assertEqual(hashlib.sha256(raw.read_bytes()).hexdigest(), self.new_profile["source_samples_sha256"])
        for profile_id in ("20260930_all12_park", "20260930_filtered9_park"):
            profile = self.registry["profiles"][profile_id]
            selected = [row for row in self.rows if row["index"] in profile["sample_indices"]]
            np.testing.assert_allclose(solve(selected), profile["tool_T_camera"]["matrix_4x4"], atol=1e-12)

    def test_old_calibration_is_exact_online_rollback(self):
        snapshot = json.loads((ROOT / "docs/calibration/tf_snapshots/20260930_before_handeye_update.json").read_text())
        previous = self.registry["profiles"]["20260831_filtered36_park"]
        for key, edge in (("tool_T_camera", "handeye_tool_frame -> camera_color_optical_frame"),
                          ("base_T_board_display", "base_link -> charuco_board_frame")):
            actual = snapshot["static"][edge]
            np.testing.assert_allclose(previous[key]["translation_xyz_m"], actual["translation_xyz_m"], atol=1e-12)
            np.testing.assert_allclose(quaternion_to_matrix(previous[key]["quaternion_xyzw"]),
                                       quaternion_to_matrix(actual["quaternion_xyzw"]), atol=1e-12)

    def test_preview_fk_matches_solver_for_every_sample(self):
        obj = HandeyeTfPreview.__new__(HandeyeTfPreview)
        geometry = self.registry["kinematics"]
        obj.zero_degrees = geometry["zero_degrees"]
        obj.ros_directions = geometry["ros_directions"]
        for attribute, key in (("j1_j2_axis_offset", "j1_j2_axis_offset_m"),
                               ("upper_arm", "upper_arm_m"), ("forearm", "forearm_m"), ("tool", "tool_m")):
            setattr(obj, attribute, geometry[key])
        for row in self.rows:
            positions = row["joint_state"]["positions"]
            np.testing.assert_allclose(obj.fk_matrix(positions), fk_tool(positions), atol=1e-12)

    def test_persistent_selection_does_not_modify_profile_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "registry.json"
            path.write_text(json.dumps(self.registry))
            activate_profile(path, "20260831_filtered36_park")
            reg, selected, _ = load_registry(path)
            self.assertEqual(selected, "20260831_filtered36_park")
            self.assertEqual(reg["profiles"], self.registry["profiles"])
            before = path.read_bytes()
            with self.assertRaises(ValueError):
                activate_profile(path, "missing-profile")
            self.assertEqual(path.read_bytes(), before)

    def test_inconsistent_transform_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "registry.json"
            reg = copy.deepcopy(self.registry)
            reg["profiles"][self.active_id]["tool_T_camera"]["translation_xyz_m"][0] += 0.01
            path.write_text(json.dumps(reg))
            with self.assertRaises(ValueError):
                load_registry(path)

    def test_real_urdf_does_not_publish_calibrated_camera_frames(self):
        path = ROOT / "src/lunar_rover_description/Rover_0/urdf/lunar_rover.urdf.xacro"
        for external in ("true", "false"):
            root = ET.fromstring(xacro.process_file(str(path), mappings={
                "use_sim": "false", "external_camera_tf": external,
            }).toxml())
            children = {joint.find("child").get("link") for joint in root.findall("joint")}
            frames = {"camera_color_optical_frame", "camera_depth_optical_frame",
                      "gemini335_color_optical_frame", "gemini335_depth_optical_frame"}
            self.assertEqual(children & frames, set() if external == "true" else frames)


if __name__ == "__main__":
    unittest.main()
