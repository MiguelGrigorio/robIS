import math

import pytest

from robis_semantic_navigation.localization_evaluator import (
    footprint_distance,
    ground_truth_in_initial_map,
    nearest_reference,
    ray_rectangle_intersection,
    surveyed_reference_observation,
)
from robis_semantic_navigation.localization_report import (
    paired_range_comparisons,
    paired_sample_comparisons,
    summarize,
    summarize_diagnostics,
)


def test_nearest_reference_selects_same_class_instance():
    objects = {
        "chair": [
            {"id": "left", "x": -2.0, "y": 0.0},
            {"id": "right", "x": 2.0, "y": 0.0},
        ]
    }
    reference, error = nearest_reference("chair", 1.8, 0.1, objects)

    assert reference["id"] == "right"
    assert error == pytest.approx(0.2236068)
    assert nearest_reference("table", 0.0, 0.0, objects) is None


def test_ground_truth_is_expressed_in_initial_robot_map_frame():
    objects = {"chair": [{"id": "target", "x": 2.0, "y": 3.0, "yaw": 0.0}]}

    transformed = ground_truth_in_initial_map(
        objects, origin_x=1.0, origin_y=1.0, origin_yaw=math.pi / 2.0
    )

    target = transformed["chair"][0]
    assert target["x"] == pytest.approx(2.0)
    assert target["y"] == pytest.approx(-1.0)
    assert target["world_x"] == 2.0
    assert target["world_y"] == 3.0
    assert target["yaw"] == pytest.approx(-math.pi / 2.0)


def test_footprint_error_separates_surface_hit_from_centroid_error():
    reference = {
        "x": 5.5, "y": -4.5, "yaw": 0.0,
        "footprint": {"shape": "rectangle", "size_x": 1.5, "size_y": 0.8},
    }

    assert footprint_distance(reference, 4.82, -4.12) == 0.0
    assert footprint_distance(reference, 4.0, -4.5) == pytest.approx(0.75)
    assert footprint_distance({"x": 0, "y": 0}, 1, 1) is None


def test_surveyed_rectangle_range_uses_intercepted_surface():
    reference = {
        "x": 5.0,
        "y": 0.0,
        "yaw": 0.0,
        "footprint": {"shape": "rectangle", "size_x": 2.0, "size_y": 1.0},
    }
    assert ray_rectangle_intersection(0.0, 0.0, 0.0, reference) == 4.0
    assert ray_rectangle_intersection(0.0, 2.0, 0.0, reference) is None

    observed = surveyed_reference_observation(
        "dining table", 0.0, (0.0, 0.0, 0.0),
        {"dining table": [reference]},
    )
    assert observed[0] == 4.0
    assert observed[1] == 0.0
    assert observed[2] == "surveyed_map_footprint"


def test_surveyed_reference_selects_instance_by_bearing():
    objects = {
        "chair": [
            {"id": "front", "x": 2.0, "y": 0.0},
            {"id": "left", "x": 0.0, "y": 3.0},
        ]
    }
    observed = surveyed_reference_observation(
        "chair", math.pi / 2.0, (0.0, 0.0, 0.0), objects
    )
    assert observed[0] == 3.0
    assert observed[1] == pytest.approx(math.pi / 2.0)
    assert observed[2] == "surveyed_map_center"


def test_report_groups_methods_and_computes_metrics():
    rows = [
        {"range_estimator": "center", "label": "chair", "trial_id": "a", "error_m": "0.2"},
        {"range_estimator": "center", "label": "chair", "trial_id": "a", "error_m": "0.6"},
        {"range_estimator": "center", "label": "chair", "trial_id": "b", "error_m": "0.8"},
        {"range_estimator": "minimum", "label": "chair", "trial_id": "a", "error_m": "1.0"},
    ]
    report = summarize(rows)

    assert len(report) == 2
    assert report[0]["mean_error_m"] == pytest.approx(0.5333333)
    assert report[0]["trials"] == 2
    assert report[0]["trial_median_mean_m"] == pytest.approx(0.6)
    assert report[0]["trial_median_error_m"] == {"a": 0.4, "b": 0.8}
    assert report[0]["footprint_samples"] == 0


def test_diagnostic_report_exposes_localization_yield_and_failures():
    rows = [
        {"range_estimator": "center", "label": "chair", "localized": "true", "failure_reason": "", "trial_id": "a", "range_error_m": "0.2", "bearing_error_rad": "0.1"},
        {"range_estimator": "center", "label": "chair", "localized": "false", "failure_reason": "no_valid_scan_range", "trial_id": "a", "range_error_m": "", "bearing_error_rad": ""},
    ]
    report = summarize_diagnostics(rows)

    assert report[0]["attempts"] == 2
    assert report[0]["localization_yield"] == pytest.approx(0.5)
    assert report[0]["failure_reasons"]["no_valid_scan_range"] == 1
    assert report[0]["trials"]["a"]["attempts"] == 2
    assert report[0]["median_range_error_m"] == pytest.approx(0.2)
    assert report[0]["trial_median_range_error_m"] == {"a": 0.2}
    assert report[0]["median_bearing_error_deg"] == pytest.approx(
        math.degrees(0.1)
    )
    assert report[0]["trial_median_bearing_error_deg"]["a"] == pytest.approx(
        math.degrees(0.1)
    )


def test_paired_report_uses_independent_trial_medians_and_exact_sign_test():
    rows = []
    for trial in ("a", "b", "c"):
        rows.extend([
            {"range_estimator": "bbox_cluster", "label": "chair",
             "trial_id": f"{trial}_bbox_cluster", "localized": "true",
             "range_error_m": "0.1"},
            {"range_estimator": "center", "label": "chair",
             "trial_id": f"{trial}_center", "localized": "true",
             "range_error_m": "0.2"},
        ])

    report = paired_range_comparisons(rows)

    assert report[0]["paired_trials"] == 3
    assert report[0]["reference_wins"] == 3
    assert report[0]["other_wins"] == 0
    assert report[0]["mean_paired_difference_m"] == pytest.approx(-0.1)
    assert report[0]["median_paired_difference_m"] == pytest.approx(-0.1)
    assert report[0]["mean_paired_difference_ci95_m"] == pytest.approx(
        [-0.1, -0.1]
    )
    assert report[0]["difference_definition"] == "reference_minus_other"
    assert report[0]["exact_two_sided_sign_p"] == pytest.approx(0.25)
    assert report[0]["trial_groups"] == 3
    assert report[0]["group_reference_wins"] == 3
    assert report[0]["group_exact_two_sided_sign_p"] == pytest.approx(0.25)


def test_paired_report_groups_repeated_trials_by_pose():
    rows = []
    for pose, differences in {
        "south": (0.4, 0.2),
        "north": (-0.1, -0.3),
    }.items():
        for repetition, difference in enumerate(differences, start=1):
            rows.extend([
                {"range_estimator": "bbox_cluster", "label": "table",
                 "trial_id": f"multi_{pose}_r{repetition:02d}_bbox_cluster",
                 "localized": "true", "range_error_m": str(1.0 + difference)},
                {"range_estimator": "mask", "label": "table",
                 "trial_id": f"multi_{pose}_r{repetition:02d}_mask",
                 "localized": "true", "range_error_m": "1.0"},
            ])

    report = paired_range_comparisons(rows)[0]

    assert report["paired_trials"] == 4
    assert report["trial_groups"] == 2
    assert report["group_paired_difference_m"] == pytest.approx({
        "multi_north": -0.2,
        "multi_south": 0.3,
    })
    assert report["group_reference_wins"] == 1
    assert report["group_other_wins"] == 1
    assert report["group_exact_two_sided_sign_p"] == 1.0


def test_paired_sample_report_uses_requested_metric():
    rows = [
        {"range_estimator": "bbox_cluster", "label": "table",
         "trial_id": "south_r01_bbox_cluster", "error_m": "2.0",
         "footprint_error_m": "1.0"},
        {"range_estimator": "mask", "label": "table",
         "trial_id": "south_r01_mask", "error_m": "1.5",
         "footprint_error_m": "0.4"},
    ]

    center = paired_sample_comparisons(rows, "error_m")[0]
    footprint = paired_sample_comparisons(rows, "footprint_error_m")[0]

    assert center["metric"] == "error_m"
    assert center["mean_paired_difference_m"] == pytest.approx(0.5)
    assert footprint["metric"] == "footprint_error_m"
    assert footprint["mean_paired_difference_m"] == pytest.approx(0.6)
