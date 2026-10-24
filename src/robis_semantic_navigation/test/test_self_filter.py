"""Independent pinhole geometry for rejecting the robot, not nearby furniture."""
from types import SimpleNamespace

import numpy as np
import pytest

from robis_semantic_navigation.fixed_cameras import Calibration
from robis_semantic_navigation.self_filter import overlaps_robot, robot_polygon, transform_matrix


def camera():
    transform = np.diag([1., -1., -1., 1.])
    transform[:3, 3] = [0., 0., 3.]
    return Calibration('1', 640, 480, np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]]),
                       np.zeros(14), transform)


def test_robot_centre_is_filtered_but_neighbouring_object_is_preserved():
    polygon = robot_polygon(camera(), np.eye(4), np.eye(2), np.zeros(2), [.6, .6, .32])
    assert overlaps_robot({'bbox': [300, 220, 340, 260]}, polygon)
    assert not overlaps_robot({'bbox': [400, 220, 440, 260]}, polygon)


def test_nonidentity_map_alignment_does_not_move_the_physical_silhouette():
    expected = robot_polygon(camera(), np.eye(4), np.eye(2), np.zeros(2), [.6, .6, .32])
    rotation = np.array([[0., -1.], [1., 0.]])
    pose = np.eye(4)
    pose[:2, :2] = rotation
    pose[:2, 3] = [2., -1.]
    actual = robot_polygon(camera(), pose, rotation, np.array([2., -1.]), [.6, .6, .32])
    assert set(map(tuple, expected.reshape(-1, 2))) == set(map(tuple, actual.reshape(-1, 2)))


def test_robot_outside_the_camera_frustum_does_not_hide_everything():
    pose = np.eye(4)
    pose[2, 3] = 5.
    assert robot_polygon(camera(), pose, np.eye(2), np.zeros(2), [.6, .6, .32]) is None
    assert not overlaps_robot({'bbox': [0, 0, 640, 480]}, None)


def test_tf_quaternion_is_normalized_and_invalid_pose_is_rejected():
    transform = SimpleNamespace(rotation=SimpleNamespace(x=0., y=0., z=0., w=2.),
                                translation=SimpleNamespace(x=1., y=2., z=0.))
    actual = transform_matrix(transform)
    assert np.allclose(actual[:3, :3], np.eye(3))
    assert np.allclose(actual[:3, 3], [1, 2, 0])
    transform.rotation.w = 0.
    with pytest.raises(ValueError):
        transform_matrix(transform)
