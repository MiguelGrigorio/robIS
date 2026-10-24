#!/usr/bin/env python3
"""Record live semantic-localization error against configured ground truth."""

import csv
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray
import yaml


FIELDS = (
    "wall_time", "ros_time", "environment", "trial_id", "range_estimator", "label",
    "confidence", "estimated_x", "estimated_y", "reference_id",
    "reference_x", "reference_y", "error_m", "footprint_error_m",
    "reference_world_x", "reference_world_y",
)
DIAGNOSTIC_FIELDS = (
    "wall_time", "ros_time", "environment", "trial_id", "range_estimator",
    "label", "confidence", "localized", "failure_reason", "bearing_rad",
    "range_m", "reference_range_m", "range_error_m", "reference_bearing_rad",
    "bearing_error_rad", "source",
    "scan_stamp", "sensor_delta_sec", "reference_source",
)


def nearest_reference(label, x, y, objects):
    """Return the nearest configured instance of ``label`` and its error."""
    candidates = objects.get(str(label), [])
    if not candidates:
        return None
    reference = min(
        candidates,
        key=lambda item: math.hypot(
            float(item["x"]) - float(x), float(item["y"]) - float(y)
        ),
    )
    return reference, math.hypot(
        float(reference["x"]) - float(x),
        float(reference["y"]) - float(y),
    )


def footprint_distance(reference, x, y):
    """Distance from a point to an optional oriented rectangular footprint."""
    footprint = reference.get("footprint") or {}
    if footprint.get("shape") != "rectangle":
        return None
    half_x = float(footprint["size_x"]) / 2.0
    half_y = float(footprint["size_y"]) / 2.0
    yaw = float(reference.get("yaw", 0.0))
    delta_x = float(x) - float(reference["x"])
    delta_y = float(y) - float(reference["y"])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    outside_x = max(abs(local_x) - half_x, 0.0)
    outside_y = max(abs(local_y) - half_y, 0.0)
    return math.hypot(outside_x, outside_y)


def ray_rectangle_intersection(origin_x, origin_y, ray_yaw, reference):
    """Return distance from a ray origin to a surveyed rectangle surface."""
    footprint = reference.get("footprint") or {}
    if footprint.get("shape") != "rectangle":
        return None
    half_x = float(footprint["size_x"]) / 2.0
    half_y = float(footprint["size_y"]) / 2.0
    object_yaw = float(reference.get("yaw", 0.0))
    cosine, sine = math.cos(object_yaw), math.sin(object_yaw)
    delta_x = float(origin_x) - float(reference["x"])
    delta_y = float(origin_y) - float(reference["y"])
    local_origin = (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
    )
    ray_x, ray_y = math.cos(float(ray_yaw)), math.sin(float(ray_yaw))
    local_direction = (
        cosine * ray_x + sine * ray_y,
        -sine * ray_x + cosine * ray_y,
    )
    near, far = -math.inf, math.inf
    for origin, direction, half_extent in (
        (local_origin[0], local_direction[0], half_x),
        (local_origin[1], local_direction[1], half_y),
    ):
        if abs(direction) < 1e-12:
            if origin < -half_extent or origin > half_extent:
                return None
            continue
        first = (-half_extent - origin) / direction
        second = (half_extent - origin) / direction
        near = max(near, min(first, second))
        far = min(far, max(first, second))
        if near > far:
            return None
    if far < 0.0:
        return None
    return max(near, 0.0)


def surveyed_reference_observation(label, bearing, lidar_pose, objects):
    """Find the surveyed instance closest to a measured lidar bearing."""
    candidates = objects.get(str(label), [])
    if not candidates or bearing is None:
        return None
    lidar_x, lidar_y, lidar_yaw = map(float, lidar_pose)
    observations = []
    for reference in candidates:
        delta_x = float(reference["x"]) - lidar_x
        delta_y = float(reference["y"]) - lidar_y
        center_bearing = math.atan2(delta_y, delta_x) - lidar_yaw
        center_bearing = math.atan2(
            math.sin(center_bearing), math.cos(center_bearing)
        )
        bearing_error = abs(math.atan2(
            math.sin(float(bearing) - center_bearing),
            math.cos(float(bearing) - center_bearing),
        ))
        surface_range = ray_rectangle_intersection(
            lidar_x, lidar_y, lidar_yaw + float(bearing), reference
        )
        observations.append((
            bearing_error, reference, center_bearing, surface_range
        ))
    _, reference, center_bearing, surface_range = min(
        observations, key=lambda item: item[0]
    )
    footprint = reference.get("footprint") or {}
    if footprint.get("shape") == "rectangle":
        source = (
            "surveyed_map_footprint" if surface_range is not None
            else "surveyed_map_ray_miss"
        )
        reference_range = surface_range
    else:
        source = "surveyed_map_center"
        reference_range = math.hypot(
            float(reference["x"]) - lidar_x,
            float(reference["y"]) - lidar_y,
        )
    return reference_range, center_bearing, source


def ground_truth_in_initial_map(objects, origin_x, origin_y, origin_yaw):
    """Transform Gazebo-world references into the SLAM map's initial frame."""
    cosine, sine = math.cos(float(origin_yaw)), math.sin(float(origin_yaw))
    transformed = {}
    for label, instances in objects.items():
        transformed[label] = []
        for instance in instances:
            item = dict(instance)
            world_x, world_y = float(item["x"]), float(item["y"])
            delta_x = world_x - float(origin_x)
            delta_y = world_y - float(origin_y)
            item["world_x"], item["world_y"] = world_x, world_y
            item["x"] = cosine * delta_x + sine * delta_y
            item["y"] = -sine * delta_x + cosine * delta_y
            item["yaw"] = math.atan2(
                math.sin(float(item.get("yaw", 0.0)) - float(origin_yaw)),
                math.cos(float(item.get("yaw", 0.0)) - float(origin_yaw)),
            )
            transformed[label].append(item)
    return transformed


class LocalizationEvaluator(Node):
    def __init__(self):
        super().__init__("localization_evaluator")
        self.declare_parameter("objects_topic", "/semantic_navigation/objects")
        self.declare_parameter("ground_truth_file", "")
        self.declare_parameter(
            "output_csv", "/tmp/robis_localization/localization_samples.csv"
        )
        self.declare_parameter("environment", "simulation")
        self.declare_parameter("trial_id", "trial_000")
        self.declare_parameter("range_estimator", "robust_median")
        self.declare_parameter("recording_enabled", False)
        self.declare_parameter("world_to_map_origin_x", 0.0)
        self.declare_parameter("world_to_map_origin_y", 0.0)
        self.declare_parameter("world_to_map_origin_yaw", 0.0)
        self.declare_parameter(
            "diagnostics_topic", "/semantic_navigation/localization_diagnostics"
        )
        self.declare_parameter("diagnostics_csv", "")
        self.declare_parameter("derive_reference_from_ground_truth", False)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("lidar_frame", "lidar_link")
        ground_truth_file = self.get_parameter("ground_truth_file").value
        if not ground_truth_file:
            raise ValueError("ground_truth_file is required")
        with open(ground_truth_file, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        ground_truth_frame = document.get("frame", "world")
        if ground_truth_frame not in ("world", "map"):
            raise ValueError("ground truth frame must be world or map")
        self.ground_truth_frame = ground_truth_frame
        self.source_objects = document.get("objects", {})
        self.tf_buffer = None
        self.tf_listener = None
        if bool(self.get_parameter("derive_reference_from_ground_truth").value):
            if self.ground_truth_frame != "map":
                raise ValueError(
                    "derived physical references require ground truth in frame map"
                )
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.output = Path(self.get_parameter("output_csv").value)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.output.open("a", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._stream, fieldnames=FIELDS)
        if self.output.stat().st_size == 0:
            self._writer.writeheader()
            self._stream.flush()
        diagnostics_value = self.get_parameter("diagnostics_csv").value
        self.diagnostics_output = (
            Path(diagnostics_value) if diagnostics_value
            else self.output.with_name(f"{self.output.stem}_diagnostics.csv")
        )
        self._diagnostics_stream = self.diagnostics_output.open(
            "a", encoding="utf-8", newline=""
        )
        self._diagnostics_writer = csv.DictWriter(
            self._diagnostics_stream, fieldnames=DIAGNOSTIC_FIELDS
        )
        if self.diagnostics_output.stat().st_size == 0:
            self._diagnostics_writer.writeheader()
            self._diagnostics_stream.flush()
        self.create_subscription(
            Detection3DArray,
            self.get_parameter("objects_topic").value,
            self._callback,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("diagnostics_topic").value,
            self._diagnostic_callback,
            50,
        )
        self.get_logger().info(f"Recording localization error in {self.output}")

    def _callback(self, message):
        if not bool(self.get_parameter("recording_enabled").value):
            return
        ros_time = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        objects = self.source_objects
        if self.ground_truth_frame == "world":
            objects = ground_truth_in_initial_map(
                objects,
                self.get_parameter("world_to_map_origin_x").value,
                self.get_parameter("world_to_map_origin_y").value,
                self.get_parameter("world_to_map_origin_yaw").value,
            )
        for detection in message.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            point = detection.bbox.center.position
            matched = nearest_reference(
                result.hypothesis.class_id, point.x, point.y, objects
            )
            if matched is None:
                continue
            reference, error = matched
            footprint_error = footprint_distance(reference, point.x, point.y)
            self._writer.writerow({
                "wall_time": f"{time.time():.6f}",
                "ros_time": f"{ros_time:.9f}",
                "environment": self.get_parameter("environment").value,
                "trial_id": self.get_parameter("trial_id").value,
                "range_estimator": self.get_parameter("range_estimator").value,
                "label": result.hypothesis.class_id,
                "confidence": f"{float(result.hypothesis.score):.6f}",
                "estimated_x": f"{float(point.x):.6f}",
                "estimated_y": f"{float(point.y):.6f}",
                "reference_id": reference.get("id", ""),
                "reference_x": f"{float(reference['x']):.6f}",
                "reference_y": f"{float(reference['y']):.6f}",
                "error_m": f"{error:.6f}",
                "footprint_error_m": (
                    f"{footprint_error:.6f}"
                    if footprint_error is not None else ""
                ),
                "reference_world_x": reference.get("world_x", ""),
                "reference_world_y": reference.get("world_y", ""),
            })
        self._stream.flush()

    def _diagnostic_callback(self, message):
        if not bool(self.get_parameter("recording_enabled").value):
            return
        try:
            item = json.loads(message.data)
        except (TypeError, ValueError):
            return
        reference_range = item.get("reference_range_m")
        reference_bearing = item.get("reference_bearing_rad")
        reference_source = item.get("reference_source")
        if (
            self.tf_buffer is not None
            and item.get("bearing_rad") is not None
        ):
            # Surveyed-map mode is authoritative. Never retain an Oracle or
            # detector-provided error when the surveyed ray misses or TF fails.
            reference_range = None
            reference_bearing = None
            range_error = None
            bearing_error = None
            stamp_sec = item.get("scan_stamp") or item.get("stamp")
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.get_parameter("map_frame").value,
                    self.get_parameter("lidar_frame").value,
                    Time(seconds=float(stamp_sec)),
                    timeout=Duration(seconds=0.1),
                )
                translation = transform.transform.translation
                orientation = transform.transform.rotation
                lidar_yaw = math.atan2(
                    2.0 * (
                        orientation.w * orientation.z
                        + orientation.x * orientation.y
                    ),
                    1.0 - 2.0 * (
                        orientation.y * orientation.y
                        + orientation.z * orientation.z
                    ),
                )
                surveyed = surveyed_reference_observation(
                    item.get("label"),
                    item.get("bearing_rad"),
                    (translation.x, translation.y, lidar_yaw),
                    self.source_objects,
                )
                if surveyed is not None:
                    reference_range, reference_bearing, reference_source = surveyed
            except (TransformException, TypeError, ValueError):
                reference_source = "surveyed_map_tf_unavailable"
        else:
            range_error = item.get("range_error_m")
            bearing_error = item.get("bearing_error_rad")
        if item.get("range_m") is not None and reference_range is not None:
            range_error = abs(float(item["range_m"]) - float(reference_range))
        if item.get("bearing_rad") is not None and reference_bearing is not None:
            bearing_error = abs(math.atan2(
                math.sin(float(item["bearing_rad"]) - float(reference_bearing)),
                math.cos(float(item["bearing_rad"]) - float(reference_bearing)),
            ))
        self._diagnostics_writer.writerow({
            "wall_time": f"{time.time():.6f}",
            "ros_time": item.get("stamp", ""),
            "environment": self.get_parameter("environment").value,
            "trial_id": self.get_parameter("trial_id").value,
            "range_estimator": item.get("range_estimator", ""),
            "label": item.get("label", ""),
            "confidence": item.get("confidence", ""),
            "localized": str(bool(item.get("localized", False))).lower(),
            "failure_reason": item.get("failure_reason", ""),
            "bearing_rad": item.get("bearing_rad", ""),
            "range_m": item.get("range_m", ""),
            "reference_range_m": (
                reference_range if reference_range is not None else ""
            ),
            "range_error_m": range_error if range_error is not None else "",
            "reference_bearing_rad": (
                reference_bearing if reference_bearing is not None else ""
            ),
            "bearing_error_rad": (
                bearing_error if bearing_error is not None else ""
            ),
            "source": item.get("source", ""),
            "scan_stamp": item.get("scan_stamp", ""),
            "sensor_delta_sec": item.get("sensor_delta_sec", ""),
            "reference_source": reference_source or "",
        })
        self._diagnostics_stream.flush()

    def destroy_node(self):
        self._stream.close()
        self._diagnostics_stream.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
