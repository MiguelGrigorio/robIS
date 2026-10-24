import json

import numpy as np
import pytest

pytest.importorskip("rclpy")

from std_msgs.msg import Header

from robis_vision.ollama_detector import parse_vlm_response
from robis_vision.oracle_detector import masked_median_depth


def test_normalized_bbox_is_converted_to_pixels():
    payload = json.dumps({
        "found": True, "label": "red chair", "confidence": 0.8,
        "bbox": [0.1, 0.2, 0.5, 0.8],
    })
    detection = parse_vlm_response(payload, 640, 480, Header())
    assert detection.bbox.size_x == pytest.approx(256.0)
    assert detection.bbox.size_y == pytest.approx(288.0)


def test_not_found_returns_none():
    assert parse_vlm_response('{"found": false}', 640, 480, Header()) is None


def test_oracle_depth_ignores_background_and_invalid_values():
    depth = np.array([[np.nan, 8.0], [2.0, 2.0]], dtype=np.float32)
    mask = np.array([[0, 0], [255, 255]], dtype=np.uint8)
    assert masked_median_depth(depth, mask) == pytest.approx(2.0)
