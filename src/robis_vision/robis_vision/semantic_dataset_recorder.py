#!/usr/bin/env python3
"""Record synchronized Gazebo RGB, semantic labels and metric depth."""

import json
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class SemanticDatasetRecorder(Node):
    def __init__(self):
        super().__init__("semantic_dataset_recorder")
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("labels_topic", "/camera/semantic/labels_map")
        self.declare_parameter("colored_labels_topic", "/camera/semantic/colored_map")
        self.declare_parameter("use_colored_labels_fallback", True)
        self.declare_parameter("depth_topic", "/camera/rgbd/depth_image")
        self.declare_parameter("output_directory", "/robis_ws/data/datasets/semantic_lab")
        self.declare_parameter("maximum_frames", 300)
        self.declare_parameter("frame_stride", 2)
        self.declare_parameter("save_depth", True)
        self.declare_parameter("synchronizer_slop_sec", 0.15)

        self.output = Path(self.get_parameter("output_directory").value).expanduser()
        directories = ["rgb", "labels"]
        if bool(self.get_parameter("save_depth").value):
            directories.append("depth")
        for name in directories:
            (self.output / name).mkdir(parents=True, exist_ok=True)
        self.manifest = self.output / "manifest.jsonl"
        self.bridge = CvBridge()
        self.seen = 0
        self.saved = 0

        subscribers = [
            Subscriber(self, Image, self.get_parameter("rgb_topic").value),
            Subscriber(self, Image, self.get_parameter("labels_topic").value),
            Subscriber(self, Image, self.get_parameter("depth_topic").value),
            Subscriber(self, Image, self.get_parameter("colored_labels_topic").value),
        ]
        self.synchronizer = ApproximateTimeSynchronizer(
            subscribers,
            queue_size=20,
            slop=float(self.get_parameter("synchronizer_slop_sec").value),
        )
        self.synchronizer.registerCallback(self._callback)
        self.get_logger().info(f"Recording synchronized extras/research data in {self.output}")

    @staticmethod
    def _stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    def _callback(self, rgb_message, labels_message, depth_message, colored_message):
        self.seen += 1
        stride = max(1, int(self.get_parameter("frame_stride").value))
        if (self.seen - 1) % stride:
            return
        maximum = int(self.get_parameter("maximum_frames").value)
        if maximum > 0 and self.saved >= maximum:
            return

        rgb = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding="bgr8")
        labels = self.bridge.imgmsg_to_cv2(labels_message, desired_encoding="passthrough")
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        colored = self.bridge.imgmsg_to_cv2(colored_message, desired_encoding="passthrough")
        if labels.ndim == 3:
            labels = labels[:, :, 0]
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        labels = np.asarray(labels)
        colored = np.asarray(colored)
        # Gazebo versions differ: some publish semantic IDs in labels_map, while
        # others leave that stream at zero but populate colored_map. Keep the
        # fallback explicit and record it, rather than silently accepting an
        # empty oracle annotation.
        used_colored_fallback = False
        if (
            bool(self.get_parameter("use_colored_labels_fallback").value)
            and not np.any(labels)
            and np.any(colored)
        ):
            labels = (np.any(colored != 0, axis=2) if colored.ndim == 3 else colored != 0).astype(np.uint8)
            used_colored_fallback = True
        if labels.dtype not in (np.uint8, np.uint16):
            labels = labels.astype(np.uint16)
        depth = np.asarray(depth, dtype=np.float32)

        stem = f"frame_{self.saved:06d}"
        if not cv2.imwrite(str(self.output / "rgb" / f"{stem}.png"), rgb):
            self.get_logger().error(f"Could not write RGB frame {stem}")
            return
        if not cv2.imwrite(str(self.output / "labels" / f"{stem}.png"), labels):
            self.get_logger().error(f"Could not write label frame {stem}")
            return
        save_depth = bool(self.get_parameter("save_depth").value)
        if save_depth:
            np.save(self.output / "depth" / f"{stem}.npy", depth)
        record = {
            "frame": stem,
            "rgb_stamp": self._stamp(rgb_message),
            "labels_stamp": self._stamp(labels_message),
            "depth_stamp": self._stamp(depth_message),
            "width": int(rgb.shape[1]),
            "height": int(rgb.shape[0]),
            "finite_depth_fraction": float(np.isfinite(depth).mean()),
            "depth_saved": save_depth,
            "label_source": "colored_map_fallback" if used_colored_fallback else "labels_map",
        }
        with self.manifest.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record) + "\n")
        self.saved += 1
        if self.saved % 25 == 0 or self.saved == maximum:
            self.get_logger().info(f"Saved {self.saved}/{maximum} synchronized frames")


def main(args=None):
    rclpy.init(args=args)
    node = SemanticDatasetRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
