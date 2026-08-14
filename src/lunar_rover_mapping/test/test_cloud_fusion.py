import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).parents[1] / "scripts" / "multisensor_cloud_fusion_node.py"
SPEC = importlib.util.spec_from_file_location("multisensor_cloud_fusion_node", SCRIPT)
FUSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FUSION)


def test_voxel_reduce_uses_confidence_weights():
    points = np.array([[0.01, 0.0, 0.0], [0.03, 0.0, 0.0]])
    reduced, values = FUSION.voxel_reduce(
        points,
        0.05,
        weights=np.array([4.0, 1.0]),
        values=np.array([100.0, 40.0]),
    )

    assert reduced.shape == (1, 3)
    assert np.isclose(reduced[0, 0], 0.014)
    assert np.isclose(values[0], 88.0)


def test_fusion_estimates_radial_bias_and_classifies_camera_points():
    lateral, vertical = np.meshgrid(
        np.linspace(-0.5, 0.5, 21),
        np.linspace(-0.3, 0.3, 13),
    )
    lidar = np.column_stack(
        (np.full(lateral.size, 2.0), lateral.ravel(), vertical.ravel())
    )
    directions = lidar / np.linalg.norm(lidar, axis=1)[:, None]
    camera = lidar + 0.05 * directions
    camera = np.vstack(
        (
            camera,
            np.array([2.15, 0.0, 0.0]),  # Close enough to contradict LiDAR.
            np.array([2.0, 1.0, 0.0]),  # Outside LiDAR sampling: complementary.
        )
    )
    params = FUSION.FusionParameters(
        min_overlap_matches=20,
        bias_smoothing=1.0,
        max_output_points=10000,
    )

    output, confidence, bias, stats = FUSION.fuse_point_sets(
        lidar,
        camera,
        np.zeros(3),
        params,
    )

    assert np.isclose(bias, -0.05, atol=0.005)
    assert stats["overlap"] >= lidar.shape[0] - 2
    assert stats["conflicts"] >= 1
    assert stats["complementary"] >= 1
    assert np.any(output[:, 1] > 0.9)
    assert confidence.shape[0] == output.shape[0]


def test_lidar_only_mode_preserves_a_valid_output():
    lidar = np.array([[0.0, 0.0, 0.0], [0.02, 0.0, 0.0], [0.2, 0.0, 0.0]])
    output, confidence, bias, stats = FUSION.fuse_point_sets(
        lidar,
        np.empty((0, 3)),
        np.zeros(3),
        FUSION.FusionParameters(),
        previous_bias=0.01,
    )

    assert output.shape[0] == 2
    assert np.allclose(confidence, 100.0)
    assert np.isclose(bias, 0.01)
    assert stats["output"] == 2
