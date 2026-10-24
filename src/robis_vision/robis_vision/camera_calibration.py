"""Read standard ROS camera-calibration YAML without inventing intrinsics."""

import math
from pathlib import Path

import yaml


def _matrix(value, key, rows, columns):
    matrix = value[key]
    if matrix.get("rows") != rows or matrix.get("cols") != columns:
        raise ValueError("Invalid dimensions for " + key)
    data = [float(item) for item in matrix["data"]]
    if len(data) != rows * columns or not all(math.isfinite(item) for item in data):
        raise ValueError("Invalid values for " + key)
    return data


def calibration_from_mapping(value, width, height):
    """Validate calibration for exactly the configured image resolution."""
    if value["image_width"] != width or value["image_height"] != height:
        raise ValueError("Camera calibration resolution does not match capture")
    k = _matrix(value, "camera_matrix", 3, 3)
    r = _matrix(value, "rectification_matrix", 3, 3)
    p = _matrix(value, "projection_matrix", 3, 4)
    coefficients = value["distortion_coefficients"]
    d = [float(item) for item in coefficients["data"]]
    if k[0] <= 0.0 or k[4] <= 0.0 or p[0] <= 0.0 or p[5] <= 0.0:
        raise ValueError("Camera focal lengths must be positive")
    if not all(math.isfinite(item) for item in d):
        raise ValueError("Invalid distortion coefficients")
    if coefficients.get("rows", 1) * coefficients.get("cols", len(d)) != len(d):
        raise ValueError("Invalid distortion coefficient dimensions")
    model = value.get("distortion_model", "plumb_bob")
    expected = {"plumb_bob": 5, "rational_polynomial": 8, "equidistant": 4}
    if model not in expected or len(d) != expected[model]:
        raise ValueError("Unsupported distortion model or coefficient count")
    return {"k": k, "r": r, "p": p, "d": d, "distortion_model": model}


def load_calibration(url, width, height):
    """Load a local file, or return None for an uncalibrated camera."""
    if not url:
        return None
    filename = url.removeprefix("file://")
    if "://" in filename:
        raise ValueError("Use a local path or file:// for camera_info_url")
    with Path(filename).expanduser().open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    return calibration_from_mapping(value, width, height)
