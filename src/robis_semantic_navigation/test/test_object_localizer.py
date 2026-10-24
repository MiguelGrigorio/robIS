from types import SimpleNamespace
import math

from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, LaserScan
from tf2_ros import Buffer
from vision_msgs.msg import Detection2D

from robis_semantic_navigation.object_localizer import (
    ObjectLocalizer,
    camera_point_to_pixel,
    detection_iou,
    nearest_stamped_message,
    pixel_matches_instance,
    project_depth_point,
    sensor_arrivals_recent,
    stamps_close,
)


def stamp(seconds, nanoseconds=0):
    return SimpleNamespace(sec=seconds, nanosec=nanoseconds)


def test_sensor_stamps_must_match_detection_time():
    assert stamps_close(stamp(10, 100_000_000), stamp(10, 200_000_000), 0.2)
    assert not stamps_close(stamp(10), stamp(11), 0.2)
    assert stamps_close(stamp(0), stamp(11), 0.2)


def test_project_aligned_depth_to_optical_point():
    info = CameraInfo()
    info.k = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]
    assert project_depth_point(420.0, 200.0, 2.0, info) == (0.5, -0.2, 2.0)
    assert camera_point_to_pixel(0.5, -0.2, 2.0, info) == (420, 200)
    assert camera_point_to_pixel(0.0, 0.0, -1.0, info) is None


def test_nearest_buffered_sensor_is_selected_for_delayed_inference():
    scans = []
    for seconds, nanoseconds in ((10, 0), (10, 100_000_000), (10, 200_000_000)):
        scan = LaserScan()
        scan.header.stamp.sec = seconds
        scan.header.stamp.nanosec = nanoseconds
        scans.append(scan)

    selected = nearest_stamped_message(scans, stamp(10, 80_000_000))

    assert selected is scans[1]


def test_instance_mask_cannot_leak_between_detections():
    mask = np.array([[0, 1, 2]], dtype=np.uint16)

    assert pixel_matches_instance(mask, 1, 0, 1)
    assert not pixel_matches_instance(mask, 2, 0, 1)
    assert pixel_matches_instance(mask, 2, 0, 2)
    assert not pixel_matches_instance(mask, 3, 0, 2)


def test_sensor_activity_uses_monotonic_arrival_not_ros_clock():
    assert sensor_arrivals_recent(100.0, 100.2, 100.5, 0.75)
    assert not sensor_arrivals_recent(99.0, 100.2, 100.5, 0.75)
    assert not sensor_arrivals_recent(None, 100.2, 100.5, 0.75)


@pytest.mark.parametrize('rotation', [0.0, 0.1])
@pytest.mark.parametrize('first_index', [0, 173])
def test_mask_projection_compensates_motion_between_scan_and_rgb(rotation, first_index):
    buffer = Buffer()
    for nanoseconds, yaw in [(0, 0), (100_000_000, rotation)]:
        transform = TransformStamped()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'lidar_link'
        transform.header.stamp.sec = 10
        transform.header.stamp.nanosec = nanoseconds
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        buffer.set_transform(transform, 'test')
    optical = TransformStamped()
    optical.header.frame_id = 'lidar_link'
    optical.child_frame_id = 'camera_optical_frame'
    optical.transform.rotation.x = -0.5
    optical.transform.rotation.y = 0.5
    optical.transform.rotation.z = -0.5
    optical.transform.rotation.w = 0.5
    buffer.set_transform_static(optical, 'test')
    scan = LaserScan()
    scan.header.frame_id = 'lidar_link'
    scan.header.stamp.sec = 10
    scan.angle_min = -(first_index + 1) * 0.01
    scan.angle_increment = 0.01
    scan.angle_max = 0.02
    scan.range_min, scan.range_max = 0.1, 10.0
    scan.ranges = [float('inf')] * first_index + [5.0, 2.0, 2.0, 5.0]
    info = CameraInfo()
    info.k = [400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0]
    detection = Detection2D()
    detection.header.frame_id = 'camera_optical_frame'
    detection.header.stamp.sec = 10
    detection.header.stamp.nanosec = 100_000_000
    pixels = [round(320 - 400 * math.tan(angle - rotation)) for angle in [0, 0.01]]
    left, right = min(pixels) - 1, max(pixels) + 1
    detection.bbox.center.position.x = (left + right) / 2
    detection.bbox.center.position.y = 240.0
    detection.bbox.size_x = float(right - left)
    detection.bbox.size_y = 4.0
    mask = np.zeros((480, 640), dtype=np.uint16)
    mask[239:242, left:right + 1] = 1
    parameters = {'motion_fixed_frame': 'odom', 'mask_pixel_tolerance_px': 0, 'minimum_scan_samples': 2,
                  'maximum_cluster_jump_m': 0.35}
    localizer = SimpleNamespace(tf_buffer=buffer, get_parameter=lambda name:
                                SimpleNamespace(value=parameters[name]))
    observation = ObjectLocalizer._mask_projected_observation(
        localizer, scan, info, mask, detection, 'lidar_link', 'camera_optical_frame',
        -rotation - 0.01, -rotation + 0.02, 1)
    assert observation == pytest.approx((2.0, 0.005))
    assert scan.header.frame_id == 'lidar_link'
    assert detection.header.frame_id == 'camera_optical_frame'


def test_mask_edge_uncertainty_never_borrows_another_instance():
    mask = np.array([[0, 0, 1, 0, 2]], dtype=np.uint16)
    assert not pixel_matches_instance(mask, 1, 0, 1)
    assert pixel_matches_instance(mask, 1, 0, 1, 1)
    assert not pixel_matches_instance(mask, 0, 0, 1, 1)
    assert not pixel_matches_instance(mask, 4, 0, 1, 2)


def test_detection_iou_matches_reference_boxes_without_affecting_estimate():
    first = Detection2D()
    first.bbox.center.position.x = 100.0
    first.bbox.center.position.y = 80.0
    first.bbox.size_x = 40.0
    first.bbox.size_y = 20.0
    identical = Detection2D()
    identical.bbox.center.position.x = 100.0
    identical.bbox.center.position.y = 80.0
    identical.bbox.size_x = 40.0
    identical.bbox.size_y = 20.0
    separate = Detection2D()
    separate.bbox.center.position.x = 200.0
    separate.bbox.center.position.y = 80.0
    separate.bbox.size_x = 40.0
    separate.bbox.size_y = 20.0

    assert detection_iou(first, identical) == 1.0
    assert detection_iou(first, separate) == 0.0
