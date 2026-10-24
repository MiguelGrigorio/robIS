import numpy as np
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

from robis_vision.depth_benchmark import (
    affine_lidar_calibration,
    depth_metrics,
    object_depth_error,
)
from robis_vision.detector_benchmark import box_iou, greedy_match
from robis_vision.detection_runtime_monitor import (
    percentile,
    summarize_runtime_samples,
)
from robis_vision.detector_evaluator import match_detections, per_class_metrics
from robis_vision.detector_report import report


def test_runtime_monitor_percentile_interpolates_and_handles_empty_input():
    assert percentile([], 50) is None
    assert percentile([1.0, 3.0], 50) == 2.0


def test_runtime_monitor_reports_throughput_age_and_bad_stamps():
    report = summarize_runtime_samples(
        arrivals=[10.0, 10.1, 10.3],
        ages_ms=[25.0, None, -2.0],
        stamps=[100.0, 0.0, 99.0],
        duration_sec=1.0,
    )
    assert report["message_count"] == 3
    assert np.isclose(report["frequency_hz"], 20.0 / 3.0)
    assert np.isclose(report["interarrival_ms_p50"], 150.0)
    assert report["message_age_ms_p50"] == 25.0
    assert report["valid_stamp_count"] == 1
    assert report["zero_stamp_count"] == 1
    assert report["future_stamp_count"] == 1


def detection(label, score, x, y, width=10.0, height=10.0):
    item = Detection2D()
    item.bbox.center.position.x = x
    item.bbox.center.position.y = y
    item.bbox.size_x = width
    item.bbox.size_y = height
    result = ObjectHypothesisWithPose()
    item.results.append(result)
    result.hypothesis.class_id = label
    result.hypothesis.score = score
    return item


def test_box_iou():
    assert box_iou((0, 0, 10, 10), (0, 0, 10, 10)) == 1.0
    assert box_iou((0, 0, 2, 2), (3, 3, 5, 5)) == 0.0


def test_greedy_match_does_not_reuse_ground_truth():
    expected = [(0, 0, 10, 10)]
    predicted = [
        ((0, 0, 10, 10), 0.9),
        ((1, 1, 9, 9), 0.8),
    ]
    matched, false_positive, ious = greedy_match(predicted, expected)
    assert matched == 1
    assert false_positive == 1
    assert ious == [1.0]


def test_live_detector_matching_is_class_aware():
    references = [detection("chair", 1.0, 10.0, 10.0)]
    predictions = [
        detection("chair", 0.9, 10.0, 10.0),
        detection("tv", 0.8, 10.0, 10.0),
    ]
    matched, false_positive, ious = match_detections(predictions, references)
    assert matched == 1
    assert false_positive == 1
    assert ious == [1.0]


def test_live_detector_exposes_per_class_false_positives_and_negatives():
    references = [
        detection("chair", 1.0, 10.0, 10.0),
        detection("bed", 1.0, 30.0, 30.0),
    ]
    predictions = [
        detection("chair", 0.9, 10.0, 10.0),
        detection("tv", 0.8, 30.0, 30.0),
    ]
    metrics = per_class_metrics(predictions, references)
    assert metrics["chair"]["true_positive"] == 1
    assert metrics["bed"]["false_negative"] == 1
    assert metrics["tv"]["false_positive"] == 1


def test_detector_report_counts_empty_prediction_as_false_negative():
    rows = [{
        "trial_id": "pose_a", "references": "1", "true_positive": "0",
        "false_positive": "0", "false_negative": "1", "matched_iou_mean": "",
    }, {
        "trial_id": "pose_a", "references": "1", "true_positive": "1",
        "false_positive": "0", "false_negative": "0", "matched_iou_mean": "0.8",
    }]
    result = report(rows)["overall"]
    assert result["recall"] == 0.5
    assert result["frame_detection_rate"] == 0.5


def test_detector_report_aggregates_exact_iou_and_per_class_metrics():
    rows = [{
        "trial_id": "pose_a", "references": "2", "true_positive": "1",
        "false_positive": "1", "false_negative": "1",
        "matched_iou_mean": "0.8", "matched_ious": "[0.8]",
        "per_class": (
            '{"bed":{"predictions":0,"references":1,"true_positive":0,'
            '"false_positive":0,"false_negative":1,"matched_ious":[]},'
            '"chair":{"predictions":1,"references":1,"true_positive":1,'
            '"false_positive":0,"false_negative":0,"matched_ious":[0.8]},'
            '"tv":{"predictions":1,"references":0,"true_positive":0,'
            '"false_positive":1,"false_negative":0,"matched_ious":[]}}'
        ),
    }]
    result = report(rows)
    assert result["classes"]["chair"]["recall"] == 1.0
    assert result["classes"]["bed"]["recall"] == 0.0
    assert result["classes"]["tv"]["precision"] == 0.0
    assert result["trials"]["pose_a"]["classes"]["chair"][
        "matched_iou_mean"
    ] == 0.8


def test_depth_metrics():
    expected = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
    metrics = depth_metrics(expected.copy(), expected)
    assert metrics["abs_rel"] == 0.0
    assert metrics["rmse_m"] == 0.0
    assert metrics["delta_1"] == 1.0


def test_sparse_affine_calibration_recovers_scale():
    expected = np.tile(np.linspace(1.0, 4.0, 200), (100, 1)).astype(np.float32)
    prediction = (expected - 0.5) / 2.0
    corrected, scale, offset = affine_lidar_calibration(prediction, expected)
    assert np.allclose(corrected, expected, atol=1e-4)
    assert abs(scale - 2.0) < 1e-4
    assert abs(offset - 0.5) < 1e-4


def test_object_depth_uses_eroded_mask_median():
    expected = np.full((30, 30), 2.0, dtype=np.float32)
    predicted = expected.copy()
    labels = np.zeros((30, 30), dtype=np.uint8)
    labels[5:25, 5:25] = 1
    predicted[5, 5:25] = 9.0
    metric = object_depth_error(predicted, expected, labels, erosion_pixels=2)
    assert metric["prediction_m"] == 2.0
    assert metric["error_m"] == 0.0
