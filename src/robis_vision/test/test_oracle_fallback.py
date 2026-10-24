import numpy as np

from robis_vision.oracle_detector import colored_fallback_labels


def test_colored_foreground_cannot_classify_multiple_classes():
    colors = np.array([[[0, 0, 0], [33, 193, 0], [5, 20, 30]]], dtype=np.uint8)
    assert colored_fallback_labels(colors, (1, 3), [1, 2]) is None


def test_explicit_single_class_world_can_use_foreground():
    colors = np.array([[[0, 0, 0], [33, 193, 0]]], dtype=np.uint8)
    assert colored_fallback_labels(colors, (1, 2), [3]).tolist() == [[0, 3]]


def test_fallback_rejects_another_sensor_resolution():
    assert colored_fallback_labels(np.ones((2, 2, 3)), (3, 3), [1]) is None
