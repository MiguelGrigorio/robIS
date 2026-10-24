#!/usr/bin/env python3
"""Convert recorded Gazebo semantic masks into Ultralytics datasets."""

import argparse
import json
from pathlib import Path
import shutil

import cv2
import numpy as np


DEFAULT_CLASSES = {
    1: "chair",
    2: "couch",
    3: "dining table",
    4: "bed",
    5: "toilet",
}


def parse_classes(specifications):
    classes = {}
    for specification in specifications:
        label, separator, name = specification.partition(":")
        if not separator or not name.strip():
            raise ValueError(f"Invalid class specification: {specification!r}")
        label_id = int(label)
        if label_id <= 0 or label_id in classes:
            raise ValueError(f"Semantic label must be unique and positive: {label_id}")
        classes[label_id] = name.strip()
    return classes


def mask_to_yolo(mask, classes, minimum_area_px=20):
    """Return YOLO rows and semantic-label counts for one integer mask."""
    if mask.ndim == 3:
        mask = mask[:, :, 0]
    height, width = mask.shape[:2]
    if not height or not width:
        raise ValueError("Empty semantic mask")
    unknown = {int(value) for value in np.unique(mask)} - {0} - set(classes)
    if unknown:
        raise ValueError(f"Unknown semantic labels in mask: {sorted(unknown)}")
    ordered_labels = sorted(classes)
    class_indices = {label: index for index, label in enumerate(ordered_labels)}
    rows = []
    counts = {classes[label]: 0 for label in ordered_labels}
    for label in ordered_labels:
        binary = np.where(mask.astype(np.int64) == label, 255, 0).astype(np.uint8)
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            if cv2.contourArea(contour) < minimum_area_px:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            x_center = (x + box_width / 2.0) / width
            y_center = (y + box_height / 2.0) / height
            rows.append(
                f"{class_indices[label]} {x_center:.8f} {y_center:.8f} "
                f"{box_width / width:.8f} {box_height / height:.8f}"
            )
            counts[classes[label]] += 1
    return rows, counts


def mask_to_yolo_segments(mask, classes, minimum_area_px=20,
                          polygon_epsilon_fraction=0.002):
    """Return YOLO segmentation polygons from one integer semantic mask."""
    if mask.ndim == 3:
        mask = mask[:, :, 0]
    height, width = mask.shape[:2]
    if not height or not width:
        raise ValueError("Empty semantic mask")
    unknown = {int(value) for value in np.unique(mask)} - {0} - set(classes)
    if unknown:
        raise ValueError(f"Unknown semantic labels in mask: {sorted(unknown)}")
    ordered_labels = sorted(classes)
    class_indices = {label: index for index, label in enumerate(ordered_labels)}
    rows = []
    counts = {classes[label]: 0 for label in ordered_labels}
    for label in ordered_labels:
        binary = np.where(mask.astype(np.int64) == label, 255, 0).astype(np.uint8)
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            if cv2.contourArea(contour) < minimum_area_px:
                continue
            epsilon = max(
                0.5,
                float(polygon_epsilon_fraction) * cv2.arcLength(contour, True),
            )
            polygon = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
            if len(polygon) < 3:
                continue
            coordinates = " ".join(
                f"{float(x) / width:.8f} {float(y) / height:.8f}"
                for x, y in polygon
            )
            rows.append(f"{class_indices[label]} {coordinates}")
            counts[classes[label]] += 1
    return rows, counts


def _fallback_frames(source):
    manifest = source / "manifest.jsonl"
    if not manifest.exists():
        return set()
    frames = set()
    for line in manifest.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        if record.get("label_source") != "labels_map":
            frames.add(str(record["frame"]))
    return frames


def convert(source, output, split, classes, prefix="", minimum_area_px=20,
            task="detect"):
    """Convert one independently collected pose/run into one fixed split."""
    if split not in ("train", "val", "test"):
        raise ValueError("split must be train, val or test")
    if task not in ("detect", "segment"):
        raise ValueError("task must be detect or segment")
    image_source = source / "rgb"
    mask_source = source / "labels"
    if not image_source.is_dir() or not mask_source.is_dir():
        raise ValueError("Source must contain rgb/ and labels/ directories")
    fallback_frames = _fallback_frames(source)
    if fallback_frames:
        raise ValueError(
            "Colored-label fallback collapses class IDs and cannot train a "
            f"multiclass detector; affected frames: {len(fallback_frames)}"
        )
    image_output = output / "images" / split
    label_output = output / "labels" / split
    image_output.mkdir(parents=True, exist_ok=True)
    label_output.mkdir(parents=True, exist_ok=True)
    prefix = prefix or source.name
    totals = {name: 0 for name in classes.values()}
    frames = 0
    empty_frames = 0
    for image_path in sorted(image_source.glob("*.png")):
        mask_path = mask_source / image_path.name
        if not mask_path.exists():
            raise ValueError(f"Missing semantic mask for {image_path.name}")
        output_stem = f"{prefix}_{image_path.stem}"
        output_image = image_output / f"{output_stem}.png"
        output_label = label_output / f"{output_stem}.txt"
        if output_image.exists() or output_label.exists():
            raise FileExistsError(f"Refusing to overwrite frame {output_stem}")
        mask = cv2.imread(str(mask_path), cv2.IMREAD_UNCHANGED)
        if mask is None:
            raise ValueError(f"Could not read {mask_path}")
        converter = mask_to_yolo_segments if task == "segment" else mask_to_yolo
        rows, counts = converter(mask, classes, minimum_area_px)
        shutil.copy2(image_path, output_image)
        output_label.write_text("\n".join(rows) + ("\n" if rows else ""), encoding="utf-8")
        for name, count in counts.items():
            totals[name] += count
        frames += 1
        empty_frames += not rows
    if not frames:
        raise ValueError("No PNG frames found")
    names = [classes[label] for label in sorted(classes)]
    dataset_yaml = output / "data.yaml"
    expected_yaml = "train: images/train\nval: images/val\ntest: images/test\nnames:\n" + "".join(
        f"  {index}: {json.dumps(name)}\n" for index, name in enumerate(names)
    )
    if dataset_yaml.exists() and dataset_yaml.read_text(encoding="utf-8") != expected_yaml:
        raise ValueError("Existing data.yaml uses a different class mapping")
    dataset_yaml.write_text(expected_yaml, encoding="utf-8")
    report = {
        "source": str(source.resolve()),
        "output": str(output.resolve()),
        "split": split,
        "prefix": prefix,
        "frames": frames,
        "empty_frames": empty_frames,
        "instances": totals,
        "minimum_area_px": minimum_area_px,
        "task": task,
        "class_mapping": {str(label): name for label, name in classes.items()},
    }
    report_path = output / f"conversion_{split}_{prefix}.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--split", choices=("train", "val", "test"), required=True)
    parser.add_argument("--prefix", default="")
    parser.add_argument("--minimum-area-px", type=int, default=20)
    parser.add_argument("--task", choices=("detect", "segment"), default="detect")
    parser.add_argument(
        "--class",
        dest="classes",
        action="append",
        default=[],
        metavar="LABEL_ID:NAME",
    )
    parsed = parser.parse_args(args=args)
    classes = parse_classes(parsed.classes) if parsed.classes else DEFAULT_CLASSES
    report = convert(
        parsed.source,
        parsed.output,
        parsed.split,
        classes,
        parsed.prefix,
        parsed.minimum_area_px,
        parsed.task,
    )
    print(json.dumps(report, indent=2))
