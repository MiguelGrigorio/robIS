#!/usr/bin/env python3
"""Fuse 2-D detections, camera intrinsics and a planar lidar in frame ``map``."""

import json
import math
from collections import deque
from copy import deepcopy
import time

from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401: registers PointStamped TF conversions
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray

from .geometry import bbox_cluster_observation, scan_range_estimate


def detection_center(detection):
    center = detection.bbox.center
    if hasattr(center, "position"):
        return float(center.position.x), float(center.position.y)
    return float(center.x), float(center.y)


def detection_width(detection):
    """Return bbox width across vision_msgs API variants."""
    return float(detection.bbox.size_x)


def detection_height(detection):
    """Return bbox height across vision_msgs API variants."""
    return float(detection.bbox.size_y)


def detection_iou(first, second):
    """Compute IoU between two ``Detection2D`` bounding boxes."""
    first_x, first_y = detection_center(first)
    second_x, second_y = detection_center(second)
    first_w, first_h = detection_width(first), detection_height(first)
    second_w, second_h = detection_width(second), detection_height(second)
    left = max(first_x - first_w / 2.0, second_x - second_w / 2.0)
    top = max(first_y - first_h / 2.0, second_y - second_h / 2.0)
    right = min(first_x + first_w / 2.0, second_x + second_w / 2.0)
    bottom = min(first_y + first_h / 2.0, second_y + second_h / 2.0)
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    union = first_w * first_h + second_w * second_h - intersection
    return intersection / max(union, 1e-9)


def stamps_close(first, second, tolerance_sec):
    """Return whether two ROS stamps are close enough for sensor fusion."""
    first_value = float(first.sec) + float(first.nanosec) / 1e9
    second_value = float(second.sec) + float(second.nanosec) / 1e9
    if first_value == 0.0 or second_value == 0.0:
        return True
    return abs(first_value - second_value) <= float(tolerance_sec)


def stamp_seconds(stamp):
    """Convert a ROS stamp to floating-point seconds."""
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def nearest_stamped_message(history, target_stamp):
    """Return the buffered message nearest to ``target_stamp``."""
    if not history:
        return None
    target = stamp_seconds(target_stamp)
    if target == 0.0:
        return history[-1]
    return min(
        history,
        key=lambda message: abs(stamp_seconds(message.header.stamp) - target),
    )


def sensor_arrivals_recent(camera_at, scan_at, now, maximum_age_sec):
    """Return whether both sensor streams arrived recently in monotonic time."""
    if camera_at is None or scan_at is None:
        return False
    return (
        float(now) - float(camera_at) <= float(maximum_age_sec)
        and float(now) - float(scan_at) <= float(maximum_age_sec)
    )


def project_depth_point(center_x, center_y, depth, camera_info):
    """Back-project an aligned optical-depth sample into the camera frame."""
    fx, fy = float(camera_info.k[0]), float(camera_info.k[4])
    cx, cy = float(camera_info.k[2]), float(camera_info.k[5])
    if fx <= 0.0 or fy <= 0.0 or not math.isfinite(depth) or depth <= 0.0:
        return None
    return (
        (float(center_x) - cx) * float(depth) / fx,
        (float(center_y) - cy) * float(depth) / fy,
        float(depth),
    )


def camera_point_to_pixel(x, y, z, camera_info):
    """Project a 3-D point in optical coordinates to integer image pixels."""
    fx, fy = float(camera_info.k[0]), float(camera_info.k[4])
    cx, cy = float(camera_info.k[2]), float(camera_info.k[5])
    if fx <= 0.0 or fy <= 0.0 or not math.isfinite(z) or z <= 0.0:
        return None
    u = fx * float(x) / float(z) + cx
    v = fy * float(y) / float(z) + cy
    distortion = list(getattr(camera_info, "d", []))
    if any(distortion) or camera_info.distortion_model == "equidistant":
        # Masks refer to the captured pixels, including lens distortion.
        point = np.array([[[float(x), float(y), float(z)]]], dtype=np.float64)
        matrix = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
        coefficients = np.asarray(distortion, dtype=np.float64)
        project = (cv2.fisheye.projectPoints
                   if camera_info.distortion_model == "equidistant" else cv2.projectPoints)
        pixels, _ = project(point, np.zeros(3), np.zeros(3), matrix, coefficients)
        u, v = pixels.reshape(-1, 2)[0]
    if not math.isfinite(u) or not math.isfinite(v):
        return None
    return int(round(u)), int(round(v))


def pixel_matches_instance(mask, u, v, instance_id, tolerance_px=0):
    """Check one instance, optionally allowing bounded mask-edge uncertainty."""
    u, v = int(u), int(v)
    if not (0 <= v < mask.shape[0] and 0 <= u < mask.shape[1]):
        return False
    if int(mask[v, u]) == int(instance_id):
        return True
    # Another positively assigned instance cannot be borrowed by dilation.
    if int(mask[v, u]) != 0:
        return False
    radius = max(0, min(8, int(tolerance_px)))
    return bool(np.any(mask[max(0, v - radius):v + radius + 1,
                            max(0, u - radius):u + radius + 1] == instance_id))


class ObjectLocalizer(Node):
    def __init__(self):
        super().__init__("object_localizer")
        self.declare_parameter("detections_topic", "/semantic_perception/detections")
        self.declare_parameter(
            "vlm_detections_topic", "/semantic_perception/vlm_detections"
        )
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter(
            "instance_mask_topic", "/semantic_perception/instance_mask"
        )
        self.declare_parameter("objects_topic", "/semantic_navigation/objects")
        self.declare_parameter(
            "perception_ready_topic", "/semantic_navigation/perception_ready"
        )
        self.declare_parameter(
            "diagnostics_topic", "/semantic_navigation/localization_diagnostics"
        )
        self.declare_parameter(
            "reference_detections_topic",
            "/semantic_perception/reference_detections",
        )
        self.declare_parameter("reference_minimum_iou", 0.25)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("lidar_frame", "lidar_link")
        self.declare_parameter("motion_fixed_frame", "odom")
        self.declare_parameter("mask_pixel_tolerance_px", 0)
        self.declare_parameter("scan_window_deg", 2.0)
        self.declare_parameter("minimum_scan_samples", 2)
        self.declare_parameter("maximum_cluster_jump_m", 0.35)
        self.declare_parameter("range_estimator", "bbox_cluster")
        self.declare_parameter("use_depth_hint", True)
        self.declare_parameter("maximum_sensor_age_sec", 0.5)
        self.declare_parameter("maximum_sensor_sync_delta_sec", 0.12)
        self.declare_parameter("sensor_history_size", 120)
        self.camera_info = None
        self.scan = None
        self.camera_info_received_at = None
        self.scan_received_at = None
        history_size = max(10, int(self.get_parameter("sensor_history_size").value))
        self.camera_info_history = deque(maxlen=history_size)
        self.scan_history = deque(maxlen=history_size)
        self.reference_history = deque(maxlen=history_size)
        self.mask_history = deque(maxlen=history_size)
        self.bridge = CvBridge()
        # Localization can occupy the default callback group continuously when
        # detections arrive faster than a CPU can fuse them.  Keep readiness in
        # an independent group so its heartbeat cannot be starved by inference.
        self.status_callback_group = MutuallyExclusiveCallbackGroup()
        self.fusion_scan_stamp = None
        self.fusion_sensor_delta = None
        self.fusion_reference_source = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            Detection3DArray, self.get_parameter("objects_topic").value, 10
        )
        self.status_publisher = self.create_publisher(
            Bool, self.get_parameter("perception_ready_topic").value, 10
        )
        self.diagnostics_publisher = self.create_publisher(
            String, self.get_parameter("diagnostics_topic").value, 50
        )
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self._camera_callback, 10
        )
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value,
            self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, self.get_parameter("instance_mask_topic").value,
            self._mask_callback, 10
        )
        self.create_subscription(
            Detection2DArray, self.get_parameter("detections_topic").value,
            self._detections_callback, 10
        )
        self.create_subscription(
            Detection2DArray, self.get_parameter("vlm_detections_topic").value,
            self._detections_callback, 10
        )
        self.create_subscription(
            Detection2DArray,
            self.get_parameter("reference_detections_topic").value,
            self._reference_callback,
            10,
        )
        self.create_timer(
            0.5,
            self._publish_status,
            callback_group=self.status_callback_group,
        )

    def _camera_callback(self, message):
        self.camera_info = message
        self.camera_info_received_at = time.monotonic()
        self.camera_info_history.append(message)

    def _scan_callback(self, message):
        self.scan = message
        self.scan_received_at = time.monotonic()
        self.scan_history.append(message)

    def _mask_callback(self, message):
        self.mask_history.append(message)

    def _mask_projected_observation(
            self, scan, camera_info, mask, detection, lidar_frame,
            camera_frame, bearing_left, bearing_right, instance_id):
        """Keep scan returns whose camera projection lies on the RGB mask."""
        center_x, center_y = detection_center(detection)
        half_width = detection_width(detection) / 2.0
        half_height = detection_height(detection) / 2.0
        left_px, right_px = center_x - half_width, center_x + half_width
        top_px, bottom_px = center_y - half_height, center_y + half_height
        gated = [float("inf")] * len(scan.ranges)
        # The RGB and scan rates differ. Compensate robot motion between
        # their capture times before testing a thin predicted object mask.
        camera_transform = self.tf_buffer.lookup_transform_full(
            camera_frame, rclpy.time.Time.from_msg(detection.header.stamp),
            lidar_frame, rclpy.time.Time.from_msg(scan.header.stamp),
            self.get_parameter("motion_fixed_frame").value,
            timeout=Duration(seconds=0.05),
        )
        for index, raw_range in enumerate(scan.ranges):
            distance = float(raw_range)
            angle = float(scan.angle_min) + index * float(scan.angle_increment)
            if (
                not math.isfinite(distance)
                or distance < scan.range_min or distance > scan.range_max
            ):
                continue
            point = PointStamped()
            point.header = deepcopy(scan.header)
            point.header.frame_id = lidar_frame
            point.point.x = distance * math.cos(angle)
            point.point.y = distance * math.sin(angle)
            point.point.z = 0.0
            camera_point = tf2_geometry_msgs.do_transform_point(point, camera_transform)
            pixel = camera_point_to_pixel(
                camera_point.point.x, camera_point.point.y,
                camera_point.point.z, camera_info,
            )
            if pixel is None:
                continue
            u, v = pixel
            if (
                u < left_px or u > right_px or v < top_px or v > bottom_px
                or not pixel_matches_instance(
                    mask, u, v, instance_id,
                    self.get_parameter("mask_pixel_tolerance_px").value,
                )
            ):
                continue
            gated[index] = distance
        # After motion compensation, accepted beam angles can fall outside
        # the bbox interval expressed at RGB time. The pixel/mask gate above
        # already supplies the appropriate angular support at scan time.
        accepted = [index for index, value in enumerate(gated) if math.isfinite(value)]
        if not accepted:
            return None
        return bbox_cluster_observation(
            gated, scan.angle_min, scan.angle_increment,
            scan.angle_min + (accepted[0] - 0.5) * scan.angle_increment,
            scan.angle_min + (accepted[-1] + 0.5) * scan.angle_increment,
            scan.range_min, scan.range_max,
            int(self.get_parameter("minimum_scan_samples").value),
            float(self.get_parameter("maximum_cluster_jump_m").value),
            raw_ranges=(scan.ranges if int(self.get_parameter("mask_pixel_tolerance_px").value) > 0 else None),
        )

    def _reference_callback(self, message):
        self.reference_history.append(message)

    def _reference_depth(self, detection, stamp, maximum_delta):
        reference_array = nearest_stamped_message(self.reference_history, stamp)
        if reference_array is None or not stamps_close(
            reference_array.header.stamp, stamp, maximum_delta
        ):
            return None
        label = detection.results[0].hypothesis.class_id
        candidates = [
            item for item in reference_array.detections
            if item.results
            and item.results[0].hypothesis.class_id == label
            and float(item.results[0].pose.pose.position.z) > 0.0
        ]
        if not candidates:
            return None
        matched = max(candidates, key=lambda item: detection_iou(detection, item))
        if detection_iou(detection, matched) < float(
            self.get_parameter("reference_minimum_iou").value
        ):
            return None
        return float(matched.results[0].pose.pose.position.z)

    def _fresh(self, stamp):
        if stamp.sec == 0 and stamp.nanosec == 0:
            return True
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(stamp)).nanoseconds / 1e9
        return abs(age) <= float(self.get_parameter("maximum_sensor_age_sec").value)

    def _sensor_streams_recent(self):
        """Check wall-time arrivals without depending on simulated `/clock`."""
        now = time.monotonic()
        maximum_age = float(
            self.get_parameter("maximum_sensor_age_sec").value
        )
        return sensor_arrivals_recent(
            self.camera_info_received_at,
            self.scan_received_at,
            now,
            maximum_age,
        )

    def _publish_status(self):
        ready = self._sensor_streams_recent()
        if ready:
            camera_frame = self.get_parameter("camera_frame").value
            lidar_frame = (
                self.scan.header.frame_id
                or self.get_parameter("lidar_frame").value
            )
            ready = self.tf_buffer.can_transform(
                lidar_frame, camera_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            ) and self.tf_buffer.can_transform(
                self.get_parameter("map_frame").value,
                lidar_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.05),
            )
        message = Bool()
        message.data = bool(ready)
        self.status_publisher.publish(message)

    def _diagnostic(self, detection, stamp, localized, reason,
                    bearing=None, distance=None, source="lidar",
                    reference_range=None, reference_bearing=None):
        if not detection.results:
            return
        result = detection.results[0]
        message = String()
        message.data = json.dumps({
            "stamp": float(stamp.sec) + float(stamp.nanosec) / 1e9,
            "label": result.hypothesis.class_id,
            "confidence": float(result.hypothesis.score),
            "range_estimator": self.get_parameter("range_estimator").value,
            "localized": bool(localized),
            "failure_reason": reason,
            "bearing_rad": bearing,
            "range_m": distance,
            "reference_range_m": reference_range,
            "range_error_m": (
                abs(float(distance) - float(reference_range))
                if distance is not None and reference_range is not None else None
            ),
            "reference_bearing_rad": reference_bearing,
            "bearing_error_rad": (
                abs(math.atan2(
                    math.sin(float(bearing) - float(reference_bearing)),
                    math.cos(float(bearing) - float(reference_bearing)),
                ))
                if bearing is not None and reference_bearing is not None else None
            ),
            "source": source,
            "scan_stamp": self.fusion_scan_stamp,
            "sensor_delta_sec": self.fusion_sensor_delta,
            "reference_source": self.fusion_reference_source,
        }, sort_keys=True)
        self.diagnostics_publisher.publish(message)

    def _detections_callback(self, message):
        self.fusion_scan_stamp = None
        self.fusion_sensor_delta = None
        self.fusion_reference_source = None
        output = Detection3DArray()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.get_parameter("map_frame").value
        if self.camera_info is None or self.scan is None:
            for detection in message.detections:
                self._diagnostic(
                    detection, message.header.stamp, False, "sensors_missing"
                )
            self.publisher.publish(output)
            return
        if not self._sensor_streams_recent():
            self.get_logger().warning("Ignoring detections: CameraInfo/LaserScan is stale", throttle_duration_sec=5.0)
            for detection in message.detections:
                self._diagnostic(
                    detection, message.header.stamp, False, "sensors_stale"
                )
            self.publisher.publish(output)
            return
        camera_info = nearest_stamped_message(
            self.camera_info_history, message.header.stamp
        )
        scan = nearest_stamped_message(self.scan_history, message.header.stamp)
        if camera_info is None or scan is None:
            for detection in message.detections:
                self._diagnostic(
                    detection, message.header.stamp, False, "sensors_missing"
                )
            self.publisher.publish(output)
            return
        maximum_delta = float(
            self.get_parameter("maximum_sensor_sync_delta_sec").value
        )
        self.fusion_scan_stamp = stamp_seconds(scan.header.stamp)
        self.fusion_sensor_delta = abs(
            stamp_seconds(message.header.stamp) - self.fusion_scan_stamp
        )
        if (
            not stamps_close(message.header.stamp, scan.header.stamp, maximum_delta)
            or not stamps_close(
                message.header.stamp, camera_info.header.stamp, maximum_delta
            )
        ):
            self.get_logger().warning(
                "Ignoring detections: image, CameraInfo and LaserScan are not synchronized",
                throttle_duration_sec=5.0,
            )
            for detection in message.detections:
                self._diagnostic(
                    detection, message.header.stamp, False, "not_synchronized"
                )
            self.publisher.publish(output)
            return
        fx, cx = float(camera_info.k[0]), float(camera_info.k[2])
        if fx <= 0:
            for detection in message.detections:
                self._diagnostic(
                    detection, message.header.stamp, False, "invalid_intrinsics"
                )
            return
        # Gazebo may scope CameraInfo frames as model/sensor paths (for example
        # ``robis/base_footprint/camera``), while robot_state_publisher exposes
        # the calibrated optical frame.  The configured frame is therefore the
        # authoritative production frame for the camera-lidar extrinsic.
        camera_frame = self.get_parameter("camera_frame").value
        lidar_frame = scan.header.frame_id or self.get_parameter("lidar_frame").value
        configured_estimator = self.get_parameter("range_estimator").value
        fusion_mask = None
        if configured_estimator == "mask_projected_cluster":
            mask_message = nearest_stamped_message(
                self.mask_history, message.header.stamp
            )
            if mask_message is not None and stamps_close(
                message.header.stamp, mask_message.header.stamp, maximum_delta
            ):
                fusion_mask = np.asarray(
                    self.bridge.imgmsg_to_cv2(
                        mask_message, desired_encoding="passthrough"
                    )
                )
        for detection_index, detection in enumerate(message.detections):
            if not detection.results:
                continue
            center_x, center_y = detection_center(detection)
            depth_hint = float(detection.results[0].pose.pose.position.z)
            if depth_hint > 0.0:
                self.fusion_reference_source = "detector_depth_hint"
            else:
                reference_depth = self._reference_depth(
                    detection, message.header.stamp, maximum_delta
                )
                if reference_depth is not None:
                    depth_hint = reference_depth
                    self.fusion_reference_source = "oracle_reference_topic"
            bearing = None
            distance = None
            reference_range = None
            reference_bearing = None
            source = "depth_hint"
            origin = PointStamped()
            origin.header = deepcopy(message.header)
            origin.header.frame_id = camera_frame
            ray = PointStamped()
            ray.header = origin.header
            ray.point.x = (center_x - cx) / fx
            ray.point.y = 0.0
            ray.point.z = 1.0
            try:
                projected = None
                if bool(self.get_parameter("use_depth_hint").value):
                    projected = project_depth_point(
                        center_x, center_y, depth_hint, camera_info
                    )
                if projected is not None:
                    point = PointStamped()
                    point.header = deepcopy(message.header)
                    point.header.frame_id = camera_frame
                    point.point.x, point.point.y, point.point.z = projected
                else:
                    source = "lidar"
                    lidar_origin = self.tf_buffer.transform(
                        origin, lidar_frame, timeout=Duration(seconds=0.1)
                    )
                    lidar_ray = self.tf_buffer.transform(
                        ray, lidar_frame, timeout=Duration(seconds=0.1)
                    )
                    bearing = math.atan2(
                        lidar_ray.point.y - lidar_origin.point.y,
                        lidar_ray.point.x - lidar_origin.point.x,
                    )
                    oracle_point = project_depth_point(
                        center_x, center_y, depth_hint, camera_info
                    )
                    if oracle_point is not None:
                        reference = PointStamped()
                        reference.header = message.header
                        reference.header.frame_id = camera_frame
                        reference.point.x, reference.point.y, reference.point.z = (
                            oracle_point
                        )
                        lidar_reference = self.tf_buffer.transform(
                            reference, lidar_frame, timeout=Duration(seconds=0.1)
                        )
                        reference_range = math.hypot(
                            lidar_reference.point.x, lidar_reference.point.y
                        )
                        reference_bearing = math.atan2(
                            lidar_reference.point.y, lidar_reference.point.x
                        )
                    estimator = configured_estimator
                    if estimator in (
                        "bbox_cluster", "bbox_cluster_bounded",
                        "mask_projected_cluster",
                    ):
                        half_width = max(0.5, detection_width(detection) / 2.0)
                        edge_bearings = []
                        for edge_x in (center_x - half_width, center_x + half_width):
                            edge_ray = PointStamped()
                            edge_ray.header = origin.header
                            edge_ray.point.x = (edge_x - cx) / fx
                            edge_ray.point.y = 0.0
                            edge_ray.point.z = 1.0
                            lidar_edge = self.tf_buffer.transform(
                                edge_ray, lidar_frame,
                                timeout=Duration(seconds=0.1),
                            )
                            edge_bearings.append(math.atan2(
                                lidar_edge.point.y - lidar_origin.point.y,
                                lidar_edge.point.x - lidar_origin.point.x,
                            ))
                        if estimator == "mask_projected_cluster":
                            if fusion_mask is None:
                                self._diagnostic(
                                    detection, message.header.stamp, False,
                                    "mask_missing_or_unsynchronized", bearing,
                                    None, "lidar_mask_projected",
                                    reference_range, reference_bearing,
                                )
                                continue
                            observation = self._mask_projected_observation(
                                scan, camera_info, fusion_mask, detection,
                                lidar_frame, camera_frame, edge_bearings[0],
                                edge_bearings[1], detection_index + 1,
                            )
                        else:
                            observation = bbox_cluster_observation(
                                scan.ranges, scan.angle_min,
                                scan.angle_increment, edge_bearings[0],
                                edge_bearings[1], scan.range_min,
                                scan.range_max,
                                int(self.get_parameter("minimum_scan_samples").value),
                                float(self.get_parameter("maximum_cluster_jump_m").value),
                                estimator == "bbox_cluster_bounded",
                            )
                        if observation is not None:
                            distance, selected_bearing = observation
                            if estimator == "mask_projected_cluster":
                                bearing = selected_bearing
                            source = f"lidar_{estimator}"
                        elif estimator == "bbox_cluster":
                            distance = scan_range_estimate(
                                scan.ranges, scan.angle_min,
                                scan.angle_increment, bearing, 0.0,
                                "center", scan.range_min,
                                scan.range_max, 1,
                            )
                            source = "lidar_center_fallback"
                        elif estimator == "bbox_cluster_bounded":
                            source = "lidar_bbox_cluster_bounded_rejected"
                        else:
                            source = "lidar_mask_projected_rejected"
                    else:
                        distance = scan_range_estimate(
                            scan.ranges, scan.angle_min,
                            scan.angle_increment, bearing,
                            math.radians(
                                float(self.get_parameter("scan_window_deg").value)
                            ),
                            estimator, scan.range_min, scan.range_max,
                            int(self.get_parameter("minimum_scan_samples").value),
                        )
                    if distance is None:
                        self._diagnostic(
                            detection, message.header.stamp, False,
                            (
                                "no_bounded_scan_cluster"
                                if estimator == "bbox_cluster_bounded"
                                else (
                                    "no_mask_projected_scan_cluster"
                                    if estimator == "mask_projected_cluster"
                                    else "no_valid_scan_range"
                                )
                            ), bearing, None, source,
                            reference_range,
                            reference_bearing,
                        )
                        continue
                    point = PointStamped()
                    point.header = deepcopy(scan.header)
                    point.header.frame_id = lidar_frame
                    point.point.x = distance * math.cos(bearing)
                    point.point.y = distance * math.sin(bearing)
                mapped = self.tf_buffer.transform(
                    point, self.get_parameter("map_frame").value,
                    timeout=Duration(seconds=0.1)
                )
            except TransformException as error:
                self.get_logger().warning(f"Semantic localization TF unavailable: {error}", throttle_duration_sec=5.0)
                self._diagnostic(
                    detection, message.header.stamp, False, "transform_unavailable",
                    bearing, distance, source,
                    reference_range,
                    reference_bearing,
                )
                continue
            localized = Detection3D()
            localized.header = output.header
            localized.results = deepcopy(detection.results)
            localized.bbox.center.position.x = mapped.point.x
            localized.bbox.center.position.y = mapped.point.y
            localized.bbox.center.position.z = mapped.point.z
            localized.bbox.center.orientation.w = 1.0
            localized.bbox.size.x = 0.1
            localized.bbox.size.y = 0.1
            localized.bbox.size.z = 0.1
            for result in localized.results:
                result.pose.pose.position = mapped.point
                result.pose.pose.orientation.w = 1.0
            output.detections.append(localized)
            self._diagnostic(
                detection, message.header.stamp, True, "", bearing, distance,
                source,
                reference_range,
                reference_bearing,
            )
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    executor.remove_node(node)
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
