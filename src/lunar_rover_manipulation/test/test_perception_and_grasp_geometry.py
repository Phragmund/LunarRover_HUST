import math
import sys
from pathlib import Path

import cv2
import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from foundationpose_result_bridge_node import (  # noqa: E402
    _quaternion_multiply,
)
from grasp_coordinator_node import GraspCoordinator  # noqa: E402
from lunar_rover_msgs.msg import TargetObservation  # noqa: E402
from target_detector_node import TargetDetector  # noqa: E402
from target_fusion_detector_node import TargetFusionDetector  # noqa: E402
from target_localizer_node import _fit_plane_normal  # noqa: E402


class _FakeMaskTensor:
    """Minimal torch-like adapter for SAM mask selection unit tests."""

    def __init__(self, array):
        self.array = array

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.array


def _sam_gate_detector():
    detector = TargetFusionDetector.__new__(TargetFusionDetector)
    detector.min_area_pixels = 20
    detector.ignore_rois = []
    detector.sam_min_seed_coverage = 0.45
    detector.sam_min_seed_iou = 0.10
    detector.sam_max_area_ratio_to_seed = 8.0
    detector.sam_max_area_fraction = 0.20
    return detector


def _coordinator_geometry():
    coordinator = GraspCoordinator.__new__(GraspCoordinator)
    coordinator.shoulder_x_m = 0.13
    coordinator.shoulder_z_m = 0.13145
    coordinator.upper_arm_m = 0.11391
    coordinator.forearm_m = 0.11000
    coordinator.tool_m = 0.15601
    coordinator.max_wall_yaw_mismatch_deg = 35.0
    coordinator.pregrasp_height_m = 0.08
    coordinator.fine_standoff_m = 0.04
    coordinator.grasp_z_offset_m = 0.015
    coordinator.lift_height_m = 0.12
    coordinator.wall_pregrasp_standoff_m = 0.08
    coordinator.wall_fine_standoff_m = 0.04
    coordinator.wall_grasp_offset_m = 0.005
    coordinator.wall_retreat_m = 0.10
    return coordinator


def test_hsv_fallback_selects_saturated_small_object():
    detector = TargetDetector.__new__(TargetDetector)
    detector.min_saturation = 105
    detector.min_value = 55
    detector.min_area_pixels = 120
    detector.max_area_fraction = 0.35
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (120, 80), (160, 120), (0, 0, 255), -1)
    result = detector._hsv_detection(image)
    assert result is not None
    mask, bbox, class_id, score = result
    assert bbox == (120, 80, 41, 41)
    assert class_id == "high_saturation_object"
    assert np.count_nonzero(mask) >= 1600
    assert score > 0.5


def test_fusion_hsv_stage_keeps_multiple_candidates_sorted():
    detector = TargetFusionDetector.__new__(TargetFusionDetector)
    detector.min_saturation = 105
    detector.min_value = 55
    detector.min_area_pixels = 20
    detector.max_area_fraction = 0.35
    detector.max_candidates = 6
    detector.ignore_rois = []
    image = np.zeros((160, 220, 3), dtype=np.uint8)
    cv2.rectangle(image, (20, 30), (42, 52), (0, 0, 255), -1)
    cv2.rectangle(image, (120, 70), (165, 112), (0, 255, 255), -1)
    proposals = detector._hsv_proposals(image)
    assert len(proposals) == 2
    assert {proposal.bbox for proposal in proposals} == {
        (20, 30, 23, 23),
        (120, 70, 46, 43),
    }


def test_fusion_ignore_roi_removes_fixed_occlusion_candidate():
    detector = TargetFusionDetector.__new__(TargetFusionDetector)
    detector.min_saturation = 105
    detector.min_value = 55
    detector.min_area_pixels = 20
    detector.max_area_fraction = 0.35
    detector.max_candidates = 6
    detector.ignore_rois = [(0, 0, 80, 80)]
    image = np.zeros((160, 220, 3), dtype=np.uint8)
    cv2.rectangle(image, (20, 30), (42, 52), (0, 0, 255), -1)
    cv2.rectangle(image, (120, 70), (165, 112), (0, 255, 255), -1)
    proposals = detector._hsv_proposals(image)
    assert len(proposals) == 1
    assert proposals[0].bbox == (120, 70, 46, 43)


def test_fusion_locked_hsv_rejects_other_saturated_colors():
    detector = TargetFusionDetector.__new__(TargetFusionDetector)
    detector.min_saturation = 105
    detector.min_value = 55
    detector.min_area_pixels = 20
    detector.max_area_fraction = 0.35
    detector.max_candidates = 6
    detector.ignore_rois = []
    detector.lock_hsv_hue_tolerance = 10.0
    detector.lock_hsv_saturation_tolerance = 55.0
    detector.lock_hsv_value_tolerance = 85.0

    image = np.zeros((160, 220, 3), dtype=np.uint8)
    cv2.rectangle(image, (20, 30), (42, 52), (0, 0, 255), -1)
    cv2.rectangle(image, (120, 70), (165, 112), (0, 255, 255), -1)
    proposals = detector._hsv_proposals(image)
    yellow = next(proposal for proposal in proposals if proposal.bbox == (120, 70, 46, 43))

    detector._remember_locked_hsv(image, yellow)
    filtered = detector._filter_locked_hsv(image, proposals)

    assert [proposal.bbox for proposal in filtered] == [(120, 70, 46, 43)]


def test_sam_gate_rejects_background_that_excludes_hsv_seed():
    detector = _sam_gate_detector()
    seed = np.zeros((100, 120), dtype=np.uint8)
    cv2.rectangle(seed, (45, 38), (58, 51), 255, -1)
    background = np.ones((100, 120), dtype=np.float32)
    background[38:52, 45:59] = 0.0

    selected = detector._select_sam_mask(
        [_FakeMaskTensor(background)], seed.shape, (25, 20, 65, 60), seed
    )

    assert selected is None


def test_sam_gate_keeps_compact_mask_supported_by_hsv_seed():
    detector = _sam_gate_detector()
    seed = np.zeros((100, 120), dtype=np.uint8)
    cv2.rectangle(seed, (45, 38), (58, 51), 255, -1)
    object_mask = np.zeros((100, 120), dtype=np.float32)
    cv2.rectangle(object_mask, (42, 35), (62, 55), 1.0, -1)

    selected = detector._select_sam_mask(
        [_FakeMaskTensor(object_mask)], seed.shape, (25, 20, 65, 60), seed
    )

    assert selected is not None
    mask, bbox = selected
    assert bbox == (42, 35, 21, 21)
    assert np.count_nonzero(mask) == 21 * 21


def test_support_plane_normal_recovers_wall_orientation():
    ys, zs = np.meshgrid(np.linspace(-0.1, 0.1, 30), np.linspace(0.2, 0.4, 30))
    points = np.column_stack(
        (np.full(ys.size, 0.55), ys.ravel(), zs.ravel())
    )
    normal = _fit_plane_normal(points)
    assert normal is not None
    assert abs(float(normal[0])) > 0.999
    assert abs(float(normal[2])) < 1e-6


def test_ground_and_frontal_wall_stages_are_reachable():
    coordinator = _coordinator_geometry()
    ground = {
        "point": np.array([0.27, 0.0, 0.05]),
        "normal": np.array([0.0, 0.0, 1.0]),
        "surface_type": TargetObservation.SURFACE_GROUND,
    }
    coordinator.reference_target = coordinator.normalized_surface(ground)
    for stage in ("pregrasp", "near", "grasp", "lift"):
        pose, pitch = coordinator.stage_pose(stage)
        assert len(coordinator.solve_planar_ik(pose, pitch)) == 4

    wall = {
        "point": np.array([0.35, 0.0, 0.22]),
        "normal": np.array([-1.0, 0.0, 0.0]),
        "surface_type": TargetObservation.SURFACE_WALL,
    }
    coordinator.reference_target = coordinator.normalized_surface(wall)
    for stage in ("pregrasp", "near", "grasp", "lift"):
        pose, pitch = coordinator.stage_pose(stage)
        assert len(coordinator.solve_planar_ik(pose, pitch)) == 4


def test_side_wall_outside_four_dof_approach_plane_is_rejected():
    coordinator = _coordinator_geometry()
    wall = {
        "point": np.array([0.35, 0.0, 0.22]),
        "normal": np.array([0.0, 1.0, 0.0]),
        "surface_type": TargetObservation.SURFACE_WALL,
    }
    try:
        coordinator.normalized_surface(wall)
    except ValueError as exc:
        assert "4-DOF" in str(exc)
    else:
        raise AssertionError("side-wall target should have been rejected")


def test_foundationpose_orientation_is_rotated_into_base_frame():
    yaw_90 = np.array([0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)])
    identity = np.array([0.0, 0.0, 0.0, 1.0])
    transformed = _quaternion_multiply(yaw_90, identity)
    assert np.allclose(transformed, yaw_90)
