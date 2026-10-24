"""Pure geometry helpers for associating image detections with a planar scan."""

import math

import numpy as np


def bbox_bearing(center_x, camera_fx, camera_cx):
    """Return the horizontal optical bearing for a pixel coordinate."""
    if camera_fx <= 0:
        raise ValueError("camera focal length must be positive")
    return math.atan2(float(center_x) - float(camera_cx), float(camera_fx))


def robust_scan_range(ranges, angle_min, angle_increment, bearing, window_rad,
                      range_min=0.0, range_max=float("inf"), minimum_samples=2):
    """Return a robust median range around a bearing, or ``None``."""
    if angle_increment == 0 or not ranges:
        return None
    center = int(round((bearing - angle_min) / angle_increment))
    radius = max(0, int(math.ceil(abs(window_rad / angle_increment))))
    values = []
    for index in range(max(0, center - radius), min(len(ranges), center + radius + 1)):
        value = float(ranges[index])
        if math.isfinite(value) and range_min <= value <= range_max:
            values.append(value)
    if len(values) < minimum_samples:
        return None
    median = float(np.median(values))
    deviations = np.abs(np.asarray(values) - median)
    mad = float(np.median(deviations))
    if mad > 0:
        filtered = np.asarray(values)[deviations <= max(3.0 * mad, 0.05)]
        if len(filtered) >= minimum_samples:
            median = float(np.median(filtered))
    return median


def scan_range_estimate(ranges, angle_min, angle_increment, bearing, window_rad,
                        estimator="robust_median", range_min=0.0,
                        range_max=float("inf"), minimum_samples=2):
    """Estimate target range with a selectable, reproducible association rule."""
    if angle_increment == 0 or not ranges:
        return None
    estimator = str(estimator).lower()
    if estimator == "robust_median":
        return robust_scan_range(
            ranges, angle_min, angle_increment, bearing, window_rad,
            range_min, range_max, minimum_samples,
        )
    center = int(round((bearing - angle_min) / angle_increment))
    radius = max(0, int(math.ceil(abs(window_rad / angle_increment))))
    if estimator == "center":
        indices = [center]
    elif estimator == "minimum":
        indices = range(
            max(0, center - radius), min(len(ranges), center + radius + 1)
        )
    else:
        raise ValueError(
            "range estimator must be center, minimum or robust_median"
        )
    values = [
        float(ranges[index])
        for index in indices
        if 0 <= index < len(ranges)
        and math.isfinite(float(ranges[index]))
        and range_min <= float(ranges[index]) <= range_max
    ]
    if not values:
        return None
    return min(values) if estimator == "minimum" else values[0]


def bbox_cluster_observation(ranges, angle_min, angle_increment, bearing_left,
                             bearing_right, range_min=0.0,
                             range_max=float("inf"), minimum_samples=2,
                             maximum_jump=0.35,
                             require_boundary_contrast=False,
                             raw_ranges=None):
    """Return range and bearing of the most central supported scan cluster.

    Invalid returns and adjacent range jumps split clusters.  The closest
    The cluster closest to the bbox centre is preferred and range breaks ties.
    Its median reduces sensitivity to a single outlier. ``None`` is returned
    when no cluster has enough support.
    """
    if angle_increment == 0 or not ranges:
        return None
    low_bearing, high_bearing = sorted((float(bearing_left), float(bearing_right)))
    first = int(math.ceil((low_bearing - angle_min) / angle_increment))
    last = int(math.floor((high_bearing - angle_min) / angle_increment))
    if angle_increment < 0:
        first = int(math.ceil((high_bearing - angle_min) / angle_increment))
        last = int(math.floor((low_bearing - angle_min) / angle_increment))
    first, last = max(0, min(first, last)), min(len(ranges) - 1, max(first, last))
    if first > last:
        return None

    clusters = []
    current = []
    for index in range(first, last + 1):
        value = float(ranges[index])
        valid = math.isfinite(value) and range_min <= value <= range_max
        if not valid or (
            current and abs(value - current[-1][1]) > maximum_jump
        ):
            if len(current) >= minimum_samples:
                clusters.append(current)
            current = []
        if valid:
            current.append((index, value))
    if len(current) >= minimum_samples:
        clusters.append(current)
    if not clusters:
        return None
    if raw_ranges is not None:
        # A predicted mask can contain small background patches. Require a
        # range discontinuity on both sides in the ORIGINAL scan, rather
        # than treating the mask's artificial gaps as physical boundaries.
        def foreground(cluster):
            distance = float(np.median([item[1] for item in cluster]))
            for neighbor in (cluster[0][0] - 1, cluster[-1][0] + 1):
                if neighbor < 0 or neighbor >= len(raw_ranges):
                    return False
                value = float(raw_ranges[neighbor])
                if (math.isfinite(value) and range_min <= value <= range_max
                        and abs(value - distance) <= maximum_jump):
                    return False
            return True
        clusters = [cluster for cluster in clusters if foreground(cluster)]
        if not clusters:
            return None
    interval_center = (first + last) / 2.0
    selected = min(
        clusters,
        key=lambda cluster: (
            float(np.median([item[1] for item in cluster]))
            if raw_ranges is not None else 0.0,
            abs(float(np.median([item[0] for item in cluster])) - interval_center),
            float(np.median([item[1] for item in cluster])),
        ),
    )
    distance = float(np.median([item[1] for item in selected]))
    if require_boundary_contrast:
        selected_first = selected[0][0]
        selected_last = selected[-1][0]

        def separated_at(index):
            if index < 0 or index >= len(ranges):
                return True
            value = float(ranges[index])
            return (
                not math.isfinite(value)
                or not range_min <= value <= range_max
                or abs(value - distance) > maximum_jump
            )

        left_bounded = selected_first > first or separated_at(first - 1)
        right_bounded = selected_last < last or separated_at(last + 1)
        # A cluster that fills the complete visual interval and continues at
        # the same range on both sides is likely a wall/background surface,
        # not the detected object intersecting the planar laser.
        if not left_bounded and not right_bounded:
            return None
    scan_index = float(np.median([item[0] for item in selected]))
    return distance, float(angle_min) + scan_index * float(angle_increment)


def bbox_cluster_range(ranges, angle_min, angle_increment, bearing_left,
                       bearing_right, range_min=0.0, range_max=float("inf"),
                       minimum_samples=2, maximum_jump=0.35,
                       require_boundary_contrast=False):
    """Return only the range component of :func:`bbox_cluster_observation`."""
    observation = bbox_cluster_observation(
        ranges, angle_min, angle_increment, bearing_left, bearing_right,
        range_min, range_max, minimum_samples, maximum_jump,
        require_boundary_contrast,
    )
    return observation[0] if observation is not None else None


def polar_to_xy(distance, bearing):
    """Convert a planar polar observation to Cartesian coordinates."""
    return distance * math.cos(bearing), distance * math.sin(bearing)


def approach_pose(robot_x, robot_y, object_x, object_y, standoff):
    """Compute a collision-free intent pose standoff metres before an object."""
    dx, dy = object_x - robot_x, object_y - robot_y
    distance = math.hypot(dx, dy)
    if distance < 1e-6:
        return robot_x, robot_y, 0.0
    yaw = math.atan2(dy, dx)
    usable = max(0.0, distance - max(0.0, standoff))
    return robot_x + usable * math.cos(yaw), robot_y + usable * math.sin(yaw), yaw


def approach_candidates(robot_x, robot_y, object_x, object_y, standoff,
                        angle_offsets_deg=(0.0, 30.0, -30.0, 60.0, -60.0,
                                           90.0, -90.0)):
    """Return poses on an arc around an object, each facing the object."""
    radius = max(0.0, float(standoff))
    object_to_robot = math.atan2(robot_y - object_y, robot_x - object_x)
    candidates = []
    for offset in angle_offsets_deg:
        radial = object_to_robot + math.radians(float(offset))
        x = object_x + radius * math.cos(radial)
        y = object_y + radius * math.sin(radial)
        yaw = math.atan2(object_y - y, object_x - x)
        candidates.append((x, y, yaw))
    return candidates


def within_standoff(robot_x, robot_y, object_x, object_y, standoff,
                    tolerance=0.25):
    """Return whether the robot already occupies an acceptable approach ring."""
    distance = math.hypot(object_x - robot_x, object_y - robot_y)
    return abs(distance - max(0.0, float(standoff))) <= max(
        0.0, float(tolerance)
    )


def nearest_neighbor_route(waypoints, start_xy):
    """Return a deterministic nearest-neighbor ordering from the current pose."""
    remaining = [dict(waypoint) for waypoint in waypoints]
    ordered = []
    current_x, current_y = start_xy
    while remaining:
        index = min(
            range(len(remaining)),
            key=lambda item: (
                math.hypot(
                    float(remaining[item]["x"]) - current_x,
                    float(remaining[item]["y"]) - current_y,
                ),
                item,
            ),
        )
        waypoint = remaining.pop(index)
        ordered.append(waypoint)
        current_x, current_y = float(waypoint["x"]), float(waypoint["y"])
    return ordered


def occupancy_coverage_waypoints(
        data, width, height, resolution, origin_x=0.0, origin_y=0.0,
        origin_yaw=0.0, spacing=1.5, clearance=0.45,
        occupancy_threshold=20, max_waypoints=40):
    """Generate deterministic observation points over safe known free space.

    Unknown cells and cells above ``occupancy_threshold`` are treated as
    obstacles. Clearance is conservatively evaluated with a square footprint;
    Nav2 remains responsible for validating actual paths and robot geometry.
    """
    width, height = int(width), int(height)
    resolution = float(resolution)
    if width <= 0 or height <= 0 or resolution <= 0:
        return []
    grid = np.asarray(data, dtype=np.int16)
    if grid.size != width * height:
        raise ValueError("occupancy data size does not match width * height")
    grid = grid.reshape((height, width))
    free = (grid >= 0) & (grid <= int(occupancy_threshold))
    blocked = (~free).astype(np.int32)

    # Summed-area table allows each clearance test to remain O(1).
    integral = np.pad(blocked, ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    radius = max(0, int(math.ceil(float(clearance) / resolution)))
    stride = max(1, int(math.ceil(float(spacing) / resolution)))
    offset = stride // 2
    candidates = []
    cosine, sine = math.cos(float(origin_yaw)), math.sin(float(origin_yaw))
    for row in range(offset, height, stride):
        for column in range(offset, width, stride):
            if (row - radius < 0 or row + radius >= height
                    or column - radius < 0 or column + radius >= width):
                continue
            row_min, row_max = max(0, row - radius), min(height - 1, row + radius)
            col_min, col_max = max(0, column - radius), min(width - 1, column + radius)
            blocked_count = (
                integral[row_max + 1, col_max + 1]
                - integral[row_min, col_max + 1]
                - integral[row_max + 1, col_min]
                + integral[row_min, col_min]
            )
            if blocked_count:
                continue
            local_x = (column + 0.5) * resolution
            local_y = (row + 0.5) * resolution
            candidates.append({
                "x": origin_x + cosine * local_x - sine * local_y,
                "y": origin_y + sine * local_x + cosine * local_y,
                "yaw": 0.0,
            })

    limit = max(0, int(max_waypoints))
    if limit == 0:
        return []
    if len(candidates) > limit:
        # Preserve coverage of the complete map instead of truncating one side.
        indices = np.linspace(0, len(candidates) - 1, limit, dtype=int)
        candidates = [candidates[index] for index in indices]
    return candidates
