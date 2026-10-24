#!/usr/bin/env python3
"""Offline metric-depth comparison against Gazebo ground truth."""

import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np


def depth_metrics(prediction, ground_truth, minimum_depth=0.1, maximum_depth=10.0):
    prediction = np.asarray(prediction, dtype=np.float32)
    ground_truth = np.asarray(ground_truth, dtype=np.float32)
    valid = (
        np.isfinite(prediction)
        & np.isfinite(ground_truth)
        & (ground_truth >= minimum_depth)
        & (ground_truth <= maximum_depth)
        & (prediction > 0.0)
    )
    if not np.any(valid):
        return None
    predicted = prediction[valid]
    expected = ground_truth[valid]
    difference = predicted - expected
    ratio = np.maximum(predicted / expected, expected / predicted)
    return {
        "pixels": int(valid.sum()),
        "abs_rel": float(np.mean(np.abs(difference) / expected)),
        "rmse_m": float(np.sqrt(np.mean(difference**2))),
        "mae_m": float(np.mean(np.abs(difference))),
        "delta_1": float(np.mean(ratio < 1.25)),
    }


def affine_lidar_calibration(prediction, ground_truth, sample_rows=3):
    """Fit scale/offset using sparse horizontal samples that emulate a 2-D lidar."""
    height, width = prediction.shape
    rows = np.linspace(height * 0.45, height * 0.60, sample_rows).astype(int)
    columns = np.linspace(0, width - 1, min(width, 180)).astype(int)
    predicted = prediction[np.ix_(rows, columns)].reshape(-1)
    expected = ground_truth[np.ix_(rows, columns)].reshape(-1)
    valid = np.isfinite(predicted) & np.isfinite(expected) & (predicted > 0) & (expected > 0)
    if valid.sum() < 10:
        return prediction, 1.0, 0.0
    design = np.column_stack([predicted[valid], np.ones(valid.sum())])
    scale, offset = np.linalg.lstsq(design, expected[valid], rcond=None)[0]
    return prediction * scale + offset, float(scale), float(offset)


def object_depth_error(
    prediction, ground_truth, labels, target_label=1, erosion_pixels=5
):
    """Compare robust depth inside an eroded semantic object mask."""
    mask = np.where(np.asarray(labels) == target_label, 255, 0).astype(np.uint8)
    if erosion_pixels > 0:
        size = erosion_pixels * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        mask = cv2.erode(mask, kernel)
    valid = (
        (mask > 0)
        & np.isfinite(prediction)
        & np.isfinite(ground_truth)
        & (prediction > 0)
        & (ground_truth > 0)
    )
    if valid.sum() < 20:
        return None
    predicted_median = float(np.median(prediction[valid]))
    expected_median = float(np.median(ground_truth[valid]))
    return {
        "prediction_m": predicted_median,
        "ground_truth_m": expected_median,
        "error_m": predicted_median - expected_median,
    }


def _load_model(backend, model_name, device):
    if backend == "yolo26":
        from ultralytics import YOLO

        return YOLO(model_name or "yolo26n-depth.pt")
    from transformers import pipeline

    return pipeline(
        "depth-estimation",
        model=model_name or "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf",
        device=-1 if device == "cpu" else int(device),
    )


def _predict(model, backend, image, device):
    if backend == "yolo26":
        result = model.predict(image, device=device, verbose=False)[0]
        return result.depth.data.detach().cpu().numpy().astype(np.float32).squeeze()
    from PIL import Image as PilImage

    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    output = model(PilImage.fromarray(rgb))
    prediction = output.get("predicted_depth", output.get("depth"))
    if hasattr(prediction, "detach"):
        prediction = prediction.detach().cpu().numpy()
    return np.asarray(prediction, dtype=np.float32).squeeze()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=["yolo26", "depth_anything"], required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", default="")
    parser.add_argument("--device", default="0")
    parser.add_argument("--calibrate-lidar", action="store_true")
    parser.add_argument("--maximum-frames", type=int, default=0)
    parser.add_argument("--target-label", type=int, default=1)
    parser.add_argument("--erosion-pixels", type=int, default=5)
    parsed = parser.parse_args(args=args)

    model = _load_model(parsed.backend, parsed.model, parsed.device)
    image_paths = sorted((parsed.dataset / "rgb").glob("*.png"))
    if parsed.maximum_frames > 0:
        image_paths = image_paths[: parsed.maximum_frames]
    frame_metrics = []
    latencies = []
    calibrations = []
    object_metrics = []
    for image_path in image_paths:
        depth_path = parsed.dataset / "depth" / f"{image_path.stem}.npy"
        label_path = parsed.dataset / "labels" / f"{image_path.stem}.png"
        if not depth_path.exists():
            continue
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        ground_truth = np.load(depth_path)
        started = time.perf_counter()
        prediction = _predict(model, parsed.backend, image, parsed.device)
        latencies.append((time.perf_counter() - started) * 1000.0)
        if prediction.shape != ground_truth.shape:
            prediction = cv2.resize(
                prediction,
                (ground_truth.shape[1], ground_truth.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            )
        if parsed.calibrate_lidar:
            prediction, scale, offset = affine_lidar_calibration(prediction, ground_truth)
            calibrations.append((scale, offset))
        metrics = depth_metrics(prediction, ground_truth)
        if metrics is not None:
            frame_metrics.append(metrics)
        labels = cv2.imread(str(label_path), cv2.IMREAD_UNCHANGED)
        if labels is not None:
            if labels.ndim == 3:
                labels = labels[:, :, 0]
            object_metric = object_depth_error(
                prediction,
                ground_truth,
                labels,
                parsed.target_label,
                parsed.erosion_pixels,
            )
            if object_metric is not None:
                object_metrics.append(object_metric)

    keys = ("abs_rel", "rmse_m", "mae_m", "delta_1")
    report = {
        "backend": parsed.backend,
        "model": parsed.model
        or (
            "yolo26n-depth.pt"
            if parsed.backend == "yolo26"
            else "Depth-Anything-V2-Metric-Indoor-Small"
        ),
        "lidar_calibrated": parsed.calibrate_lidar,
        "frames": len(frame_metrics),
        **{
            key: float(np.mean([item[key] for item in frame_metrics]))
            if frame_metrics
            else 0.0
            for key in keys
        },
        "latency_ms_p50": float(np.percentile(latencies, 50)) if latencies else 0.0,
        "latency_ms_p95": float(np.percentile(latencies, 95)) if latencies else 0.0,
        "object_frames": len(object_metrics),
        "object_depth_mae_m": float(
            np.mean([abs(item["error_m"]) for item in object_metrics])
        )
        if object_metrics
        else 0.0,
        "object_depth_bias_m": float(
            np.mean([item["error_m"] for item in object_metrics])
        )
        if object_metrics
        else 0.0,
        "object_depth_temporal_delta_m": float(
            np.mean(
                np.abs(
                    np.diff([item["prediction_m"] for item in object_metrics])
                )
            )
        )
        if len(object_metrics) > 1
        else 0.0,
    }
    if calibrations:
        report["calibration_scale_mean"] = float(np.mean([item[0] for item in calibrations]))
        report["calibration_offset_mean"] = float(np.mean([item[1] for item in calibrations]))
    parsed.output.parent.mkdir(parents=True, exist_ok=True)
    parsed.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
