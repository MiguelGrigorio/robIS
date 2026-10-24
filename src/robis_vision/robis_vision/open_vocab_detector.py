#!/usr/bin/env python3
"""Ultralytics YOLOE / YOLO-World ROS adapter."""

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


class OpenVocabularyDetector(Node):
    def __init__(self):
        super().__init__("open_vocab_detector")
        self.declare_parameter("backend", "yoloe")
        self.declare_parameter("model", "")
        self.declare_parameter("prompts", ["door", "doorway"])
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detections_topic", "/semantic_perception/detections")
        self.declare_parameter("mask_topic", "/semantic_perception/mask")
        self.declare_parameter(
            "instance_mask_topic", "/semantic_perception/instance_mask"
        )
        self.declare_parameter("debug_image_topic", "/semantic_perception/debug_image")
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("inference_hz", 5.0)
        self.declare_parameter("device", "0")
        self.declare_parameter("cpu_threads", 2)

        backend = str(self.get_parameter("backend").value)
        model_name = str(self.get_parameter("model").value)
        if not model_name:
            model_name = "yoloe-26s-seg.pt" if backend == "yoloe" else "yolov8s-worldv2.pt"
        if backend not in ("yoloe", "yolo_world"):
            raise ValueError("backend must be 'yoloe' or 'yolo_world'")
        try:
            from ultralytics import YOLOE, YOLOWorld
        except ImportError as error:
            raise RuntimeError(
                "Ultralytics is optional. Rebuild the simulation image with "
                "semantic ML dependencies."
            ) from error
        self.prompts = list(self.get_parameter("prompts").value)
        self.model = YOLOE(model_name) if backend == "yoloe" else YOLOWorld(model_name)
        names = self.model.names
        saved_prompts = list(names.values()) if isinstance(names, dict) else list(names)
        if backend == "yoloe" and saved_prompts == self.prompts:
            # Prepared weights already contain the text embeddings. This path
            # requires neither CLIP nor downloads when starting the robot.
            pass
        elif backend == "yoloe":
            self.model.set_classes(
                self.prompts, self.model.get_text_pe(self.prompts)
            )
        else:
            self.model.set_classes(self.prompts)
        self.bridge = CvBridge()
        configured_device = str(self.get_parameter("device").value)
        if configured_device.lower() == "auto":
            import torch
            self.device = "0" if torch.cuda.is_available() else "cpu"
        else:
            self.device = configured_device
        if str(self.device).lower() == "cpu":
            import torch
            cpu_threads = max(1, int(self.get_parameter("cpu_threads").value))
            torch.set_num_threads(cpu_threads)
            self.get_logger().info(
                f"Open-vocabulary CPU inference limited to {cpu_threads} threads"
            )
        self.minimum_period = 1.0 / max(float(self.get_parameter("inference_hz").value), 0.1)
        self.last_inference = 0.0
        self.backend = backend
        self.detections_pub = self.create_publisher(
            Detection2DArray, self.get_parameter("detections_topic").value, 10
        )
        self.mask_pub = self.create_publisher(
            Image, self.get_parameter("mask_topic").value, 10
        )
        self.instance_mask_pub = self.create_publisher(
            Image, self.get_parameter("instance_mask_topic").value, 10
        )
        self.debug_pub = self.create_publisher(
            Image, self.get_parameter("debug_image_topic").value, 10
        )
        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.image_callback, 10
        )
        self.get_logger().info(
            f"{backend} detector using {model_name}; prompts={self.prompts}"
        )

    def image_callback(self, msg):
        now = time.monotonic()
        if now - self.last_inference < self.minimum_period:
            return
        self.last_inference = now
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        started = time.perf_counter()
        result = self.model.predict(
            frame,
            conf=float(self.get_parameter("confidence").value),
            imgsz=int(self.get_parameter("image_size").value),
            device=self.device,
            retina_masks=True,
            verbose=False,
        )[0]
        detections = Detection2DArray()
        detections.header = msg.header
        combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        instance_mask = np.zeros(frame.shape[:2], dtype=np.uint16)
        boxes = result.boxes
        if boxes is not None:
            xyxy = boxes.xyxy.detach().cpu().numpy()
            confidences = boxes.conf.detach().cpu().numpy()
            classes = boxes.cls.detach().cpu().numpy().astype(int)
            masks = None
            if result.masks is not None:
                masks = result.masks.data.detach().cpu().numpy()
            for index, (box, confidence, class_index) in enumerate(
                zip(xyxy, confidences, classes)
            ):
                class_id = (
                    self.prompts[class_index]
                    if 0 <= class_index < len(self.prompts)
                    else str(class_index)
                )
                detections.detections.append(
                    make_detection(msg.header, class_id, confidence, *box)
                )
                if masks is not None and index < len(masks):
                    mask = cv2.resize(
                        masks[index],
                        (frame.shape[1], frame.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )
                    combined_mask[mask > 0.5] = 255
                    instance_mask[mask > 0.5] = index + 1
                cv2.rectangle(
                    frame,
                    (int(box[0]), int(box[1])),
                    (int(box[2]), int(box[3])),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    frame,
                    f"{class_id} {confidence:.2f}",
                    (int(box[0]), max(18, int(box[1]) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        cv2.putText(
            frame,
            f"{self.backend}: {elapsed_ms:.1f} ms",
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 200, 0),
            1,
        )
        mask_msg = self.bridge.cv2_to_imgmsg(combined_mask, encoding="mono8")
        mask_msg.header = msg.header
        instance_mask_msg = self.bridge.cv2_to_imgmsg(
            instance_mask, encoding="16UC1"
        )
        instance_mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)
        self.instance_mask_pub.publish(instance_mask_msg)
        self.detections_pub.publish(detections)
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenVocabularyDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
