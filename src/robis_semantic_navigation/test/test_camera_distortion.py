"""Project LiDAR points into the actual distorted camera pixels."""

from types import SimpleNamespace
import math

from robis_semantic_navigation.object_localizer import camera_point_to_pixel


def camera(model="plumb_bob", distortion=None):
    return SimpleNamespace(k=[500., 0., 320., 0., 500., 240., 0., 0., 1.],
                           distortion_model=model, d=distortion or [0.] * 5)


def test_radial_distortion_changes_lidar_mask_association():
    """A ray at (0.4,0.1) has radius squared 0.17; k1=0.2 scales by 1.034."""
    info = camera(distortion=[0.2, 0., 0., 0., 0.])
    assert camera_point_to_pixel(0.4, 0.1, 1., info) == (527, 292)
    assert camera_point_to_pixel(0.4, 0.1, 1., camera()) == (520, 290)


def test_fisheye_projection_matches_the_calibrated_pixel():
    info = camera("equidistant", [0.01, -0.005, 0., 0.])
    radius = math.hypot(0.4, 0.1)
    theta = math.atan(radius)
    distorted = theta * (1 + 0.01 * theta**2 - 0.005 * theta**4)
    expected = (round(320 + 500 * 0.4 * distorted / radius),
                round(240 + 500 * 0.1 * distorted / radius))
    assert camera_point_to_pixel(0.4, 0.1, 1., info) == expected
    # Zero fisheye coefficients still use angular, rather than pinhole, projection.
    info.d = [0.] * 4
    expected_zero = (round(320 + 500 * 0.4 * theta / radius),
                     round(240 + 500 * 0.1 * theta / radius))
    assert camera_point_to_pixel(0.4, 0.1, 1., info) == expected_zero


def test_projection_rejects_points_behind_camera_and_uncalibrated_camera():
    assert camera_point_to_pixel(0., 0., -1., camera()) is None
    info = camera()
    info.k[0] = 0.
    assert camera_point_to_pixel(0., 0., 1., info) is None
