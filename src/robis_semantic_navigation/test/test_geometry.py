import math

import pytest

from robis_semantic_navigation.geometry import (
    approach_candidates,
    approach_pose,
    bbox_bearing,
    bbox_cluster_observation,
    bbox_cluster_range,
    nearest_neighbor_route,
    occupancy_coverage_waypoints,
    polar_to_xy,
    robust_scan_range,
    scan_range_estimate,
    within_standoff,
)


def test_bbox_bearing_uses_intrinsics():
    assert bbox_bearing(320, 500, 320) == 0.0
    assert bbox_bearing(420, 500, 320) == pytest.approx(math.atan2(100, 500))


def test_scan_range_rejects_outlier_and_invalid_values():
    ranges = [float("inf")] * 11
    ranges[3:8] = [2.0, 2.02, 9.0, 1.98, 2.01]
    result = robust_scan_range(ranges, -0.5, 0.1, 0.0, 0.21, 0.1, 8.0, 2)
    assert result == pytest.approx(2.005, abs=0.02)


def test_scan_estimators_provide_reproducible_baselines():
    ranges = [4.0, 3.0, 2.0, 3.2, 4.0]
    arguments = (ranges, -0.2, 0.1, 0.0, 0.11)

    assert scan_range_estimate(*arguments, estimator="center") == 2.0
    assert scan_range_estimate(*arguments, estimator="minimum") == 2.0
    assert scan_range_estimate(
        *arguments, estimator="robust_median", minimum_samples=2
    ) == 3.2


def test_scan_estimator_rejects_unknown_method():
    with pytest.raises(ValueError):
        scan_range_estimate([1.0], 0.0, 0.1, 0.0, 0.1, "unknown")


def test_bbox_cluster_selects_robust_foreground_cluster():
    ranges = [float("inf")] * 13
    ranges[2:5] = [5.0, 5.1, 4.9]
    ranges[6:9] = [2.0, 2.1, 1.9]
    ranges[10] = 0.4  # A single short return is not enough support.

    result = bbox_cluster_range(
        ranges, -0.6, 0.1, -0.45, 0.45,
        range_min=0.1, range_max=8.0, minimum_samples=2,
    )

    assert result == pytest.approx(2.0)


def test_bbox_cluster_splits_clusters_at_range_jump():
    result = bbox_cluster_range(
        [4.0, 4.1, 1.0, 1.1], -0.2, 0.1, -0.2, 0.1,
        minimum_samples=2, maximum_jump=0.3,
    )

    assert result == pytest.approx(1.05)


def test_bbox_cluster_returns_the_selected_cluster_bearing():
    distance, bearing = bbox_cluster_observation(
        [5.0, 5.1, 2.0, 2.1], -0.2, 0.1, -0.2, 0.1,
        minimum_samples=2, maximum_jump=0.3,
    )

    assert distance == pytest.approx(2.05)
    assert bearing == pytest.approx(0.05)


def test_bbox_cluster_prefers_central_cluster_over_shorter_lateral_one():
    distance, bearing = bbox_cluster_observation(
        [1.0, 1.1, float("inf"), 4.0, 4.1, 4.0, float("inf")],
        -0.3, 0.1, -0.3, 0.3, minimum_samples=2, maximum_jump=0.3,
    )

    assert distance == pytest.approx(4.0)
    assert bearing == pytest.approx(0.1)


def test_bbox_cluster_requires_supported_valid_returns():
    assert bbox_cluster_range(
        [float("inf"), 1.0, float("nan")], -0.1, 0.1, -0.1, 0.1,
        minimum_samples=2,
    ) is None


def test_bounded_cluster_rejects_background_continuing_outside_bbox():
    ranges = [5.0] * 9
    assert bbox_cluster_observation(
        ranges, -0.4, 0.1, -0.2, 0.2,
        minimum_samples=2, require_boundary_contrast=True,
    ) is None


def test_bounded_cluster_accepts_foreground_with_one_range_edge():
    ranges = [5.0, 5.0, 2.0, 2.0, 2.0, 5.0, 5.0]
    distance, bearing = bbox_cluster_observation(
        ranges, -0.3, 0.1, -0.2, 0.2,
        minimum_samples=2, maximum_jump=0.35,
        require_boundary_contrast=True,
    )
    assert distance == pytest.approx(2.0)
    assert bearing == pytest.approx(0.0)


def test_polar_and_standoff():
    assert polar_to_xy(2.0, 0.0) == (2.0, 0.0)
    x, y, yaw = approach_pose(0.0, 0.0, 2.0, 0.0, 0.7)
    assert (x, y, yaw) == pytest.approx((1.3, 0.0, 0.0))


def test_approach_candidates_form_arc_and_face_object():
    candidates = approach_candidates(0.0, 0.0, 2.0, 0.0, 1.0, (0, 90, -90))
    assert candidates[0] == pytest.approx((1.0, 0.0, 0.0))
    assert candidates[1] == pytest.approx((2.0, -1.0, math.pi / 2.0))
    assert candidates[2] == pytest.approx((2.0, 1.0, -math.pi / 2.0))


def test_standoff_ring_accepts_only_the_configured_margin():
    assert within_standoff(0.0, 0.0, 1.1, 0.0, 1.0, 0.2)
    assert not within_standoff(0.0, 0.0, 1.4, 0.0, 1.0, 0.2)


def test_route_starts_near_robot_and_is_deterministic():
    route = [
        {"x": 0.0, "y": 0.0},
        {"x": 5.0, "y": 0.0},
        {"x": -5.0, "y": 0.0},
    ]
    ordered = nearest_neighbor_route(route, (4.0, 0.0))

    assert [item["x"] for item in ordered] == [5.0, 0.0, -5.0]
    assert route[0]["x"] == 0.0


def test_coverage_uses_only_known_free_space_with_clearance():
    grid = [0] * 49
    grid[3 * 7 + 3] = 100
    grid[0] = -1
    route = occupancy_coverage_waypoints(
        grid, 7, 7, 1.0, spacing=1.0, clearance=1.0,
        max_waypoints=100,
    )

    cells = {(int(item["x"]), int(item["y"])) for item in route}
    assert (3, 3) not in cells
    assert (2, 3) not in cells
    assert (3, 2) not in cells
    assert all(x > 0 or y > 0 for x, y in cells)


def test_coverage_transforms_cell_centres_and_limits_route():
    route = occupancy_coverage_waypoints(
        [0] * 16, 4, 4, 1.0,
        origin_x=10.0, origin_y=20.0, origin_yaw=math.pi / 2.0,
        spacing=1.0, clearance=0.0, max_waypoints=3,
    )

    assert len(route) == 3
    assert route[0]["x"] == pytest.approx(9.5)
    assert route[0]["y"] == pytest.approx(20.5)


def test_coverage_rejects_malformed_grid():
    with pytest.raises(ValueError):
        occupancy_coverage_waypoints([0], 2, 2, 1.0)


def test_uncertain_mask_rejects_wall_despite_artificial_mask_edges():
    assert bbox_cluster_observation(
        [float('inf'), 5.0, 5.0, float('inf')],
        0.0, 0.1, 0.0, 0.3, raw_ranges=[5.0] * 4,
    ) is None


def test_uncertain_mask_uses_supported_foreground_with_raw_range_boundaries():
    observation = bbox_cluster_observation(
        [float('inf'), 2.0, 2.02, float('inf'), 5.0, 5.0, float('inf')],
        0.0, 0.1, 0.0, 0.6,
        raw_ranges=[5.0, 2.0, 2.02, 5.0, 5.0, 5.0, 5.0],
    )
    assert observation == pytest.approx((2.01, 0.15))
