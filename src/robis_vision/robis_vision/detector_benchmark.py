#!/usr/bin/env python3
"""Offline Ultralytics detector benchmark for Gazebo-labelled frames."""

import argparse
import json
from pathlib import Path
import statistics
import time

import cv2
import numpy as np


def box_iou(a, b):
    left = max(a[0], b[0])
    top = max(a[1], b[1])
    right = min(a[2], b[2])
    bottom = min(a[3], b[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return intersection / max(area_a + area_b - intersection, 1e-9)


def ground_truth_boxes(mask_path, label):
    mask = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
    if mask is None:
        return []
    if mask.ndim == 3:
        mask = mask[:, :, 0]
    binary = np.where(mask == label, 255, 0).astype(np.uint8)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for contour in contours:
        if cv2.contourArea(contour) < 20:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        boxes.append((x, y, x + width, y + height))
    return boxes


def greedy_match(predicted, expected, iou_threshold=0.5):
    """Match confidence-sorted predictions to each ground-truth box once."""
    matched_expected = set()
    matched_ious = []
    false_positive = 0
    for box, _score in sorted(predicted, key=lambda item: item[1], reverse=True):
        candidates = [
            (box_iou(box, candidate), index)
            for index, candidate in enumerate(expected)
            if index not in matched_expected
        ]
        iou, index = max(candidates, default=(0.0, -1))
        if iou >= iou_threshold:
            matched_expected.add(index)
            matched_ious.append(iou)
        else:
            false_positive += 1
    return len(matched_expected), false_positive, matched_ious


def class_name(names, class_index):
    if isinstance(names, dict):
        return str(names.get(class_index, class_index))
    return str(names[class_index])


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--backend", choices=["yolo26", "yoloe", "yolo_world"], required=True
    )
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", default="")
    parser.add_argument("--prompts", nargs="+", default=["door", "doorway"])
    parser.add_argument("--target-class", default="chair")
    parser.add_argument("--target-label", type=int, default=1)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--iou-threshold", type=float, default=0.5)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--warmup-runs", type=int, default=5)
    parsed = parser.parse_args(args=args)

    from ultralytics import YOLO, YOLOWorld

    default_models = {
        "yolo26": "yolo26n.pt",
        "yoloe": "yoloe-26s-seg.pt",
        "yolo_world": "yolov8s-worldv2.pt",
    }
    model_name = parsed.model or default_models[parsed.backend]
    model = YOLOWorld(model_name) if parsed.backend == "yolo_world" else YOLO(
        model_name
    )
    if parsed.backend == "yoloe" and hasattr(model, "get_text_pe"):
        model.set_classes(parsed.prompts, model.get_text_pe(parsed.prompts))
    elif parsed.backend == "yolo_world":
        model.set_classes(parsed.prompts)
    image_paths = sorted(
        path
        for path in parsed.images.iterdir()
        if path.suffix.lower() in (".png", ".jpg", ".jpeg")
    )
    try:
        import torch
    except ImportError:
        torch = None
    uses_cuda = (
        torch is not None
        and torch.cuda.is_available()
        and str(parsed.device).lower() != "cpu"
    )

    def synchronize():
        if uses_cuda:
            torch.cuda.synchronize()

    if image_paths:
        for _ in range(max(0, parsed.warmup_runs)):
            model.predict(
                str(image_paths[0]), conf=parsed.confidence,
                imgsz=parsed.image_size, device=parsed.device, verbose=False,
            )
        synchronize()
        if uses_cuda:
            torch.cuda.reset_peak_memory_stats()
    latencies = []
    true_positive = false_positive = false_negative = 0
    previous_box = None
    temporal_ious = []
    matched_ious = []
    frames_with_ground_truth = 0
    frames_with_detection = 0
    evaluated = 0
    for image_path in image_paths:
        label_path = parsed.labels / f"{image_path.stem}.png"
        if not label_path.exists():
            continue
        expected = ground_truth_boxes(label_path, parsed.target_label)
        synchronize()
        started = time.perf_counter()
        result = model.predict(
            str(image_path), conf=parsed.confidence, imgsz=parsed.image_size,
            device=parsed.device, verbose=False
        )[0]
        synchronize()
        latencies.append((time.perf_counter() - started) * 1000.0)
        predicted = []
        if result.boxes is not None:
            boxes = result.boxes.xyxy.detach().cpu().numpy().tolist()
            scores = result.boxes.conf.detach().cpu().numpy().tolist()
            classes = result.boxes.cls.detach().cpu().numpy().astype(int)
            predicted = [
                (box, score)
                for box, score, index in zip(boxes, scores, classes)
                if class_name(result.names, index) == parsed.target_class
            ]
        matched, frame_fp, frame_ious = greedy_match(
            predicted, expected, parsed.iou_threshold
        )
        true_positive += matched
        false_positive += frame_fp
        false_negative += len(expected) - matched
        matched_ious.extend(frame_ious)
        frames_with_ground_truth += bool(expected)
        frames_with_detection += bool(predicted)
        current_box = max(predicted, key=lambda item: item[1])[0] if predicted else None
        if previous_box is not None and current_box is not None:
            temporal_ious.append(box_iou(previous_box, current_box))
        previous_box = current_box
        evaluated += 1

    precision = true_positive / max(true_positive + false_positive, 1)
    recall = true_positive / max(true_positive + false_negative, 1)
    report = {
        "backend": parsed.backend,
        "model": model_name,
        "prompts": parsed.prompts,
        "target_class": parsed.target_class,
        "target_label": parsed.target_label,
        "confidence_threshold": parsed.confidence,
        "iou_threshold": parsed.iou_threshold,
        "image_size": parsed.image_size,
        "device": parsed.device,
        "warmup_runs": max(0, parsed.warmup_runs),
        "frames": evaluated,
        "frames_with_ground_truth": frames_with_ground_truth,
        "frames_with_detection": frames_with_detection,
        "true_positive": true_positive,
        "false_positive": false_positive,
        "false_negative": false_negative,
        "precision": precision,
        "recall": recall,
        "f1": 2 * precision * recall / max(precision + recall, 1e-9),
        "matched_iou_mean": statistics.mean(matched_ious) if matched_ious else 0.0,
        "temporal_iou_mean": statistics.mean(temporal_ious) if temporal_ious else 0.0,
        "latency_ms_p50": float(np.percentile(latencies, 50)) if latencies else 0.0,
        "latency_ms_p95": float(np.percentile(latencies, 95)) if latencies else 0.0,
    }
    report["peak_vram_mb"] = (
        torch.cuda.max_memory_allocated() / (1024 * 1024) if uses_cuda else 0.0
    )
    if torch is not None:
        report["torch_version"] = torch.__version__
        report["cuda_version"] = torch.version.cuda
    parsed.output.parent.mkdir(parents=True, exist_ok=True)
    parsed.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
