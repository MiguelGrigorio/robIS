#!/usr/bin/env python3
"""Continuous RGB detector using pretrained or adapted Ultralytics weights."""

import time

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


class Yolo26Detector(Node):
    def __init__(self):
        super().__init__("yolo26_detector")
        self.declare_parameter("model", "yolo26n.pt")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/semantic_perception/detections")
        self.declare_parameter("debug_image_topic", "/semantic_perception/debug_image")
        self.declare_parameter("mask_topic", "/semantic_perception/mask")
        self.declare_parameter(
            "instance_mask_topic", "/semantic_perception/instance_mask"
        )
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("inference_hz", 5.0)
        self.declare_parameter("device", "auto")
        self.declare_parameter("cpu_threads", 2)
        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError("Install ultralytics or use detector_backend:=oracle") from error
        model_name = str(self.get_parameter("model").value)
        self.model = YOLO(model_name)
        configured_device = str(self.get_parameter("device").value)
        if configured_device.lower() == "auto":
            import torch
            self.device = "0" if torch.cuda.is_available() else "cpu"
            self.get_logger().info(
                f"YOLO26 auto-selected device={self.device} "
                f"(CUDA available={torch.cuda.is_available()})"
            )
        else:
            self.device = configured_device
        if str(self.device).lower() == "cpu":
            import torch
            cpu_threads = max(1, int(self.get_parameter("cpu_threads").value))
            torch.set_num_threads(cpu_threads)
            self.get_logger().info(
                f"YOLO26 CPU inference limited to {cpu_threads} threads"
            )
        self.bridge = CvBridge()
        self.minimum_period = 1.0 / max(float(self.get_parameter("inference_hz").value), 0.1)
        self.last_inference = 0.0
        self.publisher = self.create_publisher(
            Detection2DArray, self.get_parameter("detections_topic").value, 10
        )
        self.debug_publisher = self.create_publisher(
            Image, self.get_parameter("debug_image_topic").value, 10
        )
        self.mask_publisher = self.create_publisher(
            Image, self.get_parameter("mask_topic").value, 10
        )
        self.instance_mask_publisher = self.create_publisher(
            Image, self.get_parameter("instance_mask_topic").value, 10
        )
        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self._image_callback, 10
        )
        self.get_logger().info(f"Ultralytics RGB detector ready: {model_name}")

    def _image_callback(self, message):
        now = time.monotonic()
        if now - self.last_inference < self.minimum_period:
            return
        self.last_inference = now
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        result = self.model.predict(
            frame, conf=float(self.get_parameter("confidence").value),
            imgsz=int(self.get_parameter("image_size").value),
            device=self.device, retina_masks=True, verbose=False
        )[0]
        output = Detection2DArray()
        output.header = message.header
        combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        instance_mask = np.zeros(frame.shape[:2], dtype=np.uint16)
        if result.boxes is not None:
            boxes = result.boxes.xyxy.detach().cpu().numpy()
            scores = result.boxes.conf.detach().cpu().numpy()
            classes = result.boxes.cls.detach().cpu().numpy().astype(int)
            names = result.names
            masks = None
            if result.masks is not None:
                masks = result.masks.data.detach().cpu().numpy()
            for index, (box, score, class_index) in enumerate(
                zip(boxes, scores, classes)
            ):
                label = names.get(class_index, str(class_index)) if isinstance(names, dict) else names[class_index]
                output.detections.append(make_detection(message.header, label, score, *box))
                cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {score:.2f}", (int(box[0]), max(16, int(box[1]) - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                if masks is not None and index < len(masks):
                    mask = cv2.resize(
                        masks[index], (frame.shape[1], frame.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )
                    combined_mask[mask > 0.5] = 255
                    # Value N identifies detection N-1 in Detection2DArray.
                    # This prevents one object's box from accepting lidar
                    # returns projected onto another object's mask.
                    instance_mask[mask > 0.5] = index + 1
        mask_message = self.bridge.cv2_to_imgmsg(combined_mask, encoding="mono8")
        mask_message.header = message.header
        instance_mask_message = self.bridge.cv2_to_imgmsg(
            instance_mask, encoding="16UC1"
        )
        instance_mask_message.header = message.header
        # Publish the same-stamp mask first so a single-threaded localizer can
        # buffer it before handling the corresponding detections.
        self.mask_publisher.publish(mask_message)
        self.instance_mask_publisher.publish(instance_mask_message)
        self.publisher.publish(output)
        debug = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug.header = message.header
        self.debug_publisher.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = Yolo26Detector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
