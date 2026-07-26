import math

from lunar_rover_localization.transforms import (
    compose_transform,
    inverse_transform,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


def test_transform_inverse_round_trip():
    transform = ((1.2, -0.4, 0.8), quaternion_from_yaw(0.7))
    identity = compose_transform(transform, inverse_transform(transform))
    assert all(abs(value) < 1e-9 for value in identity[0])
    assert abs(identity[1][0]) < 1e-9
    assert abs(identity[1][1]) < 1e-9
    assert abs(identity[1][2]) < 1e-9
    assert abs(identity[1][3] - 1.0) < 1e-9


def test_yaw_round_trip():
    for yaw in (-math.pi, -0.8, 0.0, 1.3, math.pi):
        recovered = yaw_from_quaternion(quaternion_from_yaw(yaw))
        assert abs(math.atan2(math.sin(recovered - yaw), math.cos(recovered - yaw))) < 1e-9

