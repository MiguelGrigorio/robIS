#!/usr/bin/env python3
"""Measure live detector throughput, jitter and stamped-message age."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


def percentile(values, percentile_value):
    """Return a linearly interpolated percentile without NumPy."""
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * float(percentile_value) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize_runtime_samples(arrivals, ages_ms, stamps, duration_sec):
    """Aggregate independent callback observations into runtime metrics."""
    intervals_ms = [
        (current - previous) * 1000.0
        for previous, current in zip(arrivals, arrivals[1:])
        if current >= previous
    ]
    observed_span = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0.0
    frequency_hz = (
        (len(arrivals) - 1) / observed_span if observed_span > 0.0 else 0.0
    )
    stamp_regressions = sum(
        current < previous
        for previous, current in zip(stamps, stamps[1:])
        if previous > 0.0 and current > 0.0
    )
    valid_ages = [value for value in ages_ms if value is not None and value >= 0.0]
    return {
        "requested_duration_sec": float(duration_sec),
        "message_count": len(arrivals),
        "observed_span_sec": observed_span,
        "frequency_hz": frequency_hz,
        "interarrival_ms_p50": percentile(intervals_ms, 50),
        "interarrival_ms_p95": percentile(intervals_ms, 95),
        "message_age_ms_p50": percentile(valid_ages, 50),
        "message_age_ms_p95": percentile(valid_ages, 95),
        "valid_stamp_count": len(valid_ages),
        "zero_stamp_count": sum(value <= 0.0 for value in stamps),
        "future_stamp_count": sum(
            value is not None and value < 0.0 for value in ages_ms
        ),
        "stamp_regression_count": stamp_regressions,
    }


class DetectionRuntimeMonitor(Node):
    def __init__(self, topic):
        super().__init__("detection_runtime_monitor")
        self.arrivals = []
        self.ages_ms = []
        self.stamps = []
        self.create_subscription(Detection2DArray, topic, self._callback, 50)

    def _callback(self, message):
        arrival = time.monotonic()
        now_sec = self.get_clock().now().nanoseconds / 1e9
        stamp_sec = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        age_ms = None if stamp_sec <= 0.0 else (now_sec - stamp_sec) * 1000.0
        self.arrivals.append(arrival)
        self.ages_ms.append(age_ms)
        self.stamps.append(stamp_sec)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/semantic_perception/detections")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--output", required=True)
    parsed = parser.parse_args(args=args)
    if parsed.duration <= 0.0:
        parser.error("--duration must be positive")
    output = Path(parsed.output)
    if output.exists():
        parser.error("refusing to overwrite " + str(output))

    rclpy.init()
    node = DetectionRuntimeMonitor(parsed.topic)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < parsed.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    elapsed = time.monotonic() - started
    report = summarize_runtime_samples(
        node.arrivals, node.ages_ms, node.stamps, parsed.duration
    )
    report.update({
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "topic": parsed.topic,
        "elapsed_sec": elapsed,
        "metric_definition": {
            "message_age_ms": (
                "ROS receive time minus Detection2DArray header stamp; this is "
                "end-to-end age only when the detector preserves the source-image stamp"
            ),
            "frequency_hz": "callback throughput over first-to-last arrival span",
        },
    })
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if report["message_count"] < 2:
        raise SystemExit("fewer than two detection messages were received")


if __name__ == "__main__":
    main()
