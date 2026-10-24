#!/usr/bin/env python3
"""Evaluate live detector boxes against an isolated Oracle reference topic."""

import csv
import json
from pathlib import Path
import time

from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

from .detector_benchmark import box_iou


FIELDS = (
    "wall_time", "ros_time", "trial_id", "predictions", "references",
    "true_positive", "false_positive", "false_negative", "matched_iou_mean",
    "matched_ious", "predicted_labels", "reference_labels", "per_class",
)


def detection_box(detection):
    center = detection.bbox.center
    if hasattr(center, "position"):
        center_x, center_y = center.position.x, center.position.y
    else:
        center_x, center_y = center.x, center.y
    half_width = float(detection.bbox.size_x) / 2.0
    half_height = float(detection.bbox.size_y) / 2.0
    return (
        float(center_x) - half_width,
        float(center_y) - half_height,
        float(center_x) + half_width,
        float(center_y) + half_height,
    )


def detection_label(detection):
    if not detection.results:
        return ""
    return str(detection.results[0].hypothesis.class_id)


def detection_score(detection):
    if not detection.results:
        return 0.0
    return float(detection.results[0].hypothesis.score)


def match_detections(predictions, references, iou_threshold=0.5):
    """Greedily match class-aware predictions to references exactly once."""
    matched_references = set()
    matched_ious = []
    false_positive = 0
    ordered = sorted(predictions, key=detection_score, reverse=True)
    for prediction in ordered:
        label = detection_label(prediction)
        candidates = [
            (box_iou(detection_box(prediction), detection_box(reference)), index)
            for index, reference in enumerate(references)
            if index not in matched_references
            and detection_label(reference) == label
        ]
        iou, index = max(candidates, default=(0.0, -1))
        if iou >= float(iou_threshold):
            matched_references.add(index)
            matched_ious.append(iou)
        else:
            false_positive += 1
    return len(matched_references), false_positive, matched_ious


def per_class_metrics(predictions, references, iou_threshold=0.5):
    """Return frame-level matching metrics independently for every label."""
    labels = sorted({
        detection_label(item) for item in [*predictions, *references]
        if detection_label(item)
    })
    metrics = {}
    for label in labels:
        label_predictions = [
            item for item in predictions if detection_label(item) == label
        ]
        label_references = [
            item for item in references if detection_label(item) == label
        ]
        matched, false_positive, ious = match_detections(
            label_predictions, label_references, iou_threshold
        )
        metrics[label] = {
            "predictions": len(label_predictions),
            "references": len(label_references),
            "true_positive": matched,
            "false_positive": false_positive,
            "false_negative": len(label_references) - matched,
            "matched_ious": ious,
        }
    return metrics


class DetectorEvaluator(Node):
    def __init__(self):
        super().__init__("detector_evaluator")
        self.declare_parameter(
            "predictions_topic", "/semantic_perception/detections"
        )
        self.declare_parameter(
            "references_topic", "/semantic_perception/reference_detections"
        )
        self.declare_parameter(
            "output_csv", "/tmp/robis_localization/detector_samples.csv"
        )
        self.declare_parameter("trial_id", "trial_000")
        self.declare_parameter("recording_enabled", False)
        self.declare_parameter("iou_threshold", 0.5)
        self.declare_parameter("synchronizer_slop_sec", 0.12)

        self.output = Path(self.get_parameter("output_csv").value)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.output.open("a", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(self.stream, fieldnames=FIELDS)
        if self.output.stat().st_size == 0:
            self.writer.writeheader()
            self.stream.flush()

        predictions = Subscriber(
            self, Detection2DArray,
            self.get_parameter("predictions_topic").value,
        )
        references = Subscriber(
            self, Detection2DArray,
            self.get_parameter("references_topic").value,
        )
        synchronizer = ApproximateTimeSynchronizer(
            [predictions, references], queue_size=100,
            slop=float(self.get_parameter("synchronizer_slop_sec").value),
        )
        synchronizer.registerCallback(self._callback)
        self.synchronizer = synchronizer
        self.get_logger().info(f"Recording detector evaluation in {self.output}")

    def _callback(self, predictions, references):
        if not bool(self.get_parameter("recording_enabled").value):
            return
        matched, false_positive, ious = match_detections(
            predictions.detections, references.detections,
            self.get_parameter("iou_threshold").value,
        )
        class_metrics = per_class_metrics(
            predictions.detections, references.detections,
            self.get_parameter("iou_threshold").value,
        )
        stamp = predictions.header.stamp
        self.writer.writerow({
            "wall_time": f"{time.time():.6f}",
            "ros_time": f"{stamp.sec + stamp.nanosec / 1e9:.9f}",
            "trial_id": self.get_parameter("trial_id").value,
            "predictions": len(predictions.detections),
            "references": len(references.detections),
            "true_positive": matched,
            "false_positive": false_positive,
            "false_negative": len(references.detections) - matched,
            "matched_iou_mean": sum(ious) / len(ious) if ious else "",
            "matched_ious": json.dumps(ious),
            "predicted_labels": json.dumps([
                detection_label(item) for item in predictions.detections
            ]),
            "reference_labels": json.dumps([
                detection_label(item) for item in references.detections
            ]),
            "per_class": json.dumps(class_metrics, sort_keys=True),
        })
        self.stream.flush()

    def destroy_node(self):
        self.stream.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectorEvaluator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
