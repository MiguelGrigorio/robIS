#!/usr/bin/env python3
"""Aggregate frame-level detector-evaluator CSV into reproducible metrics."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def row_ious(row):
    """Read exact matched IoUs, retaining compatibility with older CSVs."""
    serialized = row.get("matched_ious")
    if serialized not in (None, ""):
        return [float(value) for value in json.loads(serialized)]
    mean = row.get("matched_iou_mean")
    return [float(mean)] if mean not in (None, "") else []


def summarize(rows):
    true_positive = sum(int(row["true_positive"]) for row in rows)
    false_positive = sum(int(row["false_positive"]) for row in rows)
    false_negative = sum(int(row["false_negative"]) for row in rows)
    precision = true_positive / max(true_positive + false_positive, 1)
    recall = true_positive / max(true_positive + false_negative, 1)
    ious = [value for row in rows for value in row_ious(row)]
    reference_frames = sum(int(row["references"]) > 0 for row in rows)
    detected_reference_frames = sum(
        int(row["references"]) > 0 and int(row["true_positive"]) > 0
        for row in rows
    )
    return {
        "frames": len(rows),
        "reference_frames": reference_frames,
        "detected_reference_frames": detected_reference_frames,
        "frame_detection_rate": (
            detected_reference_frames / max(reference_frames, 1)
        ),
        "true_positive": true_positive,
        "false_positive": false_positive,
        "false_negative": false_negative,
        "precision": precision,
        "recall": recall,
        "f1": 2.0 * precision * recall / max(precision + recall, 1e-9),
        "matched_iou_mean": float(np.mean(ious)) if ious else None,
        "matched_iou_p50": float(np.percentile(ious, 50)) if ious else None,
    }


def class_rows(rows):
    """Expand per-frame JSON metrics into rows consumable by ``summarize``."""
    expanded = {}
    for row in rows:
        serialized = row.get("per_class")
        if serialized in (None, ""):
            continue
        for label, metrics in json.loads(serialized).items():
            item = {
                "trial_id": row.get("trial_id", "legacy"),
                "predictions": metrics["predictions"],
                "references": metrics["references"],
                "true_positive": metrics["true_positive"],
                "false_positive": metrics["false_positive"],
                "false_negative": metrics["false_negative"],
                "matched_ious": json.dumps(metrics.get("matched_ious", [])),
                "matched_iou_mean": "",
            }
            expanded.setdefault(label, []).append(item)
    return expanded


def summarize_classes(rows):
    return {
        label: summarize(items)
        for label, items in sorted(class_rows(rows).items())
    }


def report(rows):
    trials = sorted({row.get("trial_id") or "legacy" for row in rows})
    return {
        "overall": summarize(rows),
        "classes": summarize_classes(rows),
        "trials": {
            trial: {
                **summarize([
                    row for row in rows
                    if (row.get("trial_id") or "legacy") == trial
                ]),
                "classes": summarize_classes([
                    row for row in rows
                    if (row.get("trial_id") or "legacy") == trial
                ]),
            }
            for trial in trials
        },
    }


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parsed = parser.parse_args(args=args)
    with parsed.input.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    document = {
        "input": str(parsed.input),
        **report(rows),
    }
    parsed.output.parent.mkdir(parents=True, exist_ok=True)
    parsed.output.write_text(json.dumps(document, indent=2), encoding="utf-8")
    print(json.dumps(document, indent=2))
