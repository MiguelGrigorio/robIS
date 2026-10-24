#!/usr/bin/env python3
"""Optional monocular metric-depth ROS adapter."""

import time

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthEstimator(Node):
    def __init__(self):
        super().__init__("depth_estimator")
        self.declare_parameter("backend", "yolo26")
        self.declare_parameter("model", "")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("depth_topic", "/semantic_perception/depth")
        self.declare_parameter("debug_topic", "/semantic_perception/depth_debug")
        self.declare_parameter("inference_hz", 2.0)
        self.declare_parameter("device", "0")
        self.backend = str(self.get_parameter("backend").value)
        model_name = str(self.get_parameter("model").value)
        if self.backend == "yolo26":
            from ultralytics import YOLO

            self.model = YOLO(model_name or "yolo26n-depth.pt")
        elif self.backend == "depth_anything":
            from PIL import Image as PilImage
            from transformers import pipeline

            self.pil_image = PilImage
            self.model = pipeline(
                "depth-estimation",
                model=model_name
                or "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf",
                device=0 if str(self.get_parameter("device").value) != "cpu" else -1,
            )
        else:
            raise ValueError("backend must be 'yolo26' or 'depth_anything'")
        self.bridge = CvBridge()
        self.minimum_period = 1.0 / max(float(self.get_parameter("inference_hz").value), 0.1)
        self.last_inference = 0.0
        self.depth_pub = self.create_publisher(
            Image, self.get_parameter("depth_topic").value, 10
        )
        self.debug_pub = self.create_publisher(
            Image, self.get_parameter("debug_topic").value, 10
        )
        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.image_callback, 10
        )

    def image_callback(self, msg):
        now = time.monotonic()
        if now - self.last_inference < self.minimum_period:
            return
        self.last_inference = now
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.backend == "yolo26":
            result = self.model.predict(
                frame,
                device=str(self.get_parameter("device").value),
                verbose=False,
            )[0]
            depth = result.depth.data.detach().cpu().numpy().astype(np.float32)
        else:
            prediction = self.model(
                self.pil_image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            )
            raw = prediction.get("predicted_depth", prediction.get("depth"))
            if hasattr(raw, "detach"):
                raw = raw.detach().cpu().numpy()
            depth = np.asarray(raw, dtype=np.float32).squeeze()
        if depth.shape != frame.shape[:2]:
            depth = cv2.resize(
                depth,
                (frame.shape[1], frame.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            )
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header = msg.header
        self.depth_pub.publish(depth_msg)
        finite = depth[np.isfinite(depth)]
        if finite.size:
            low, high = np.percentile(finite, (2, 98))
            normalized = np.clip((depth - low) / max(high - low, 1e-6), 0, 1)
            colored = cv2.applyColorMap((normalized * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            debug_msg = self.bridge.cv2_to_imgmsg(colored, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
