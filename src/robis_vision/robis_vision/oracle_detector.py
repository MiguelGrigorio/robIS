#!/usr/bin/env python3
"""Turn Gazebo semantic labels into deterministic ROS detections."""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from .detection_utils import make_detection


def masked_median_depth(depth, mask):
    """Return the median finite positive depth for one semantic component."""
    if depth is None or depth.shape[:2] != mask.shape[:2]:
        return None
    values = np.asarray(depth, dtype=np.float32)[mask.astype(bool)]
    values = values[np.isfinite(values) & (values > 0.0)]
    if values.size == 0:
        return None
    return float(np.median(values))


def colored_fallback_labels(colored, shape, label_ids):
    """Identify foreground only for a world with one explicitly configured class."""
    if len(label_ids) != 1 or colored.shape[:2] != shape:
        return None
    foreground = np.any(colored != 0, axis=2) if colored.ndim == 3 else colored != 0
    return foreground.astype(np.int64) * int(label_ids[0])


class OracleDetector(Node):
    def __init__(self):
        super().__init__("oracle_detector")
        self.declare_parameter("labels_topic", "/camera/semantic/labels_map")
        self.declare_parameter("colored_labels_topic", "/camera/semantic/colored_map")
        self.declare_parameter("detections_topic", "/semantic_perception/detections")
        self.declare_parameter("mask_topic", "/semantic_perception/mask")
        self.declare_parameter(
            "instance_mask_topic", "/semantic_perception/instance_mask"
        )
        self.declare_parameter("depth_topic", "/camera/rgbd/depth_image")
        self.declare_parameter("target_label", 1)
        self.declare_parameter("target_name", "door")
        self.declare_parameter("label_ids", [1])
        self.declare_parameter("label_names", ["chair"])
        self.declare_parameter("minimum_area_px", 200)
        self.declare_parameter("use_colored_labels_fallback", True)
        self.bridge = CvBridge()
        self.detections_pub = self.create_publisher(
            Detection2DArray, self.get_parameter("detections_topic").value, 10
        )
        self.mask_pub = self.create_publisher(
            Image, self.get_parameter("mask_topic").value, 10
        )
        self.instance_mask_pub = self.create_publisher(
            Image, self.get_parameter("instance_mask_topic").value, 10
        )
        self.create_subscription(
            Image, self.get_parameter("labels_topic").value, self.labels_callback, 10
        )
        self.depth = None
        self.depth_stamp = None
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self.depth_callback, 10
        )
        self.colored_labels = None
        self.create_subscription(
            Image,
            self.get_parameter("colored_labels_topic").value,
            self.colored_labels_callback,
            10,
        )

    def colored_labels_callback(self, msg):
        self.colored_labels = msg

    def depth_callback(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.depth_stamp = msg.header.stamp

    def _depth_is_synchronized(self, stamp):
        if self.depth_stamp is None:
            return False
        first = float(stamp.sec) + float(stamp.nanosec) / 1e9
        second = float(self.depth_stamp.sec) + float(self.depth_stamp.nanosec) / 1e9
        return first == 0.0 or second == 0.0 or abs(first - second) <= 0.2

    def labels_callback(self, msg):
        labels = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if labels.ndim == 3:
            labels = labels[:, :, 0]
        if (
            bool(self.get_parameter("use_colored_labels_fallback").value)
            and not np.any(labels)
            and self.colored_labels is not None
        ):
            colored = self.bridge.imgmsg_to_cv2(
                self.colored_labels, desired_encoding="passthrough"
            )
            first = msg.header.stamp
            second = self.colored_labels.header.stamp
            delta = abs(first.sec - second.sec + (first.nanosec - second.nanosec) / 1e9)
            fallback = colored_fallback_labels(
                colored, labels.shape[:2], list(self.get_parameter("label_ids").value)
            )
            if fallback is not None and delta <= 0.2:
                labels = fallback
        output = Detection2DArray()
        output.header = msg.header
        combined_mask = np.zeros(labels.shape[:2], dtype=np.uint8)
        instance_mask = np.zeros(labels.shape[:2], dtype=np.uint16)
        minimum_area = int(self.get_parameter("minimum_area_px").value)
        label_ids = list(self.get_parameter("label_ids").value)
        label_names = list(self.get_parameter("label_names").value)
        if len(label_ids) != len(label_names):
            self.get_logger().error("label_ids and label_names must have equal length")
            return
        for label_id, label_name in zip(label_ids, label_names):
            mask = np.where(labels.astype(np.int64) == label_id, 255, 0).astype(np.uint8)
            combined_mask = cv2.bitwise_or(combined_mask, mask)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                if cv2.contourArea(contour) < minimum_area:
                    continue
                x, y, width, height = cv2.boundingRect(contour)
                detection = make_detection(
                    msg.header, label_name, 1.0, x, y, x + width, y + height
                )
                if self._depth_is_synchronized(msg.header.stamp):
                    component_mask = np.zeros(labels.shape[:2], dtype=np.uint8)
                    cv2.drawContours(component_mask, [contour], -1, 255, thickness=-1)
                    component_mask = cv2.bitwise_and(component_mask, mask)
                    depth = masked_median_depth(self.depth, component_mask)
                    if depth is not None:
                        # Positive Z is an Oracle-only optical-depth hint. The
                        # localizer still validates camera, lidar and TF health.
                        detection.results[0].pose.pose.position.z = depth
                output.detections.append(detection)
                component_mask = np.zeros(labels.shape[:2], dtype=np.uint8)
                cv2.drawContours(component_mask, [contour], -1, 255, thickness=-1)
                component_mask = cv2.bitwise_and(component_mask, mask)
                instance_mask[component_mask != 0] = len(output.detections)
        mask_msg = self.bridge.cv2_to_imgmsg(combined_mask, encoding="mono8")
        mask_msg.header = msg.header
        instance_mask_msg = self.bridge.cv2_to_imgmsg(
            instance_mask, encoding="16UC1"
        )
        instance_mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)
        self.instance_mask_pub.publish(instance_mask_msg)
        self.detections_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = OracleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
