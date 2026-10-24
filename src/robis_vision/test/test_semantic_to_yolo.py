import json

import cv2
import numpy as np
import pytest

from robis_vision.semantic_to_yolo import (
    convert,
    mask_to_yolo,
    mask_to_yolo_segments,
    parse_classes,
)


def test_mask_to_yolo_preserves_class_order_and_instances():
    mask = np.zeros((100, 200), dtype=np.uint8)
    mask[10:30, 20:60] = 1
    mask[50:90, 100:180] = 5
    rows, counts = mask_to_yolo(mask, {1: "chair", 5: "toilet"}, 10)
    assert rows == [
        "0 0.20000000 0.20000000 0.20000000 0.20000000",
        "1 0.70000000 0.70000000 0.40000000 0.40000000",
    ]
    assert counts == {"chair": 1, "toilet": 1}


def test_mask_to_yolo_rejects_unknown_labels():
    mask = np.array([[0, 2]], dtype=np.uint8)
    with pytest.raises(ValueError, match="Unknown semantic labels"):
        mask_to_yolo(mask, {1: "chair"})


def test_mask_to_yolo_segments_emits_normalized_polygon():
    mask = np.zeros((100, 200), dtype=np.uint8)
    mask[10:30, 20:60] = 1
    rows, counts = mask_to_yolo_segments(mask, {1: "chair"}, 10)
    fields = rows[0].split()
    coordinates = [float(value) for value in fields[1:]]
    assert fields[0] == "0"
    assert len(coordinates) >= 6 and len(coordinates) % 2 == 0
    assert all(0.0 <= value <= 1.0 for value in coordinates)
    assert counts == {"chair": 1}


def test_converter_rejects_colored_fallback_for_multiclass(tmp_path):
    source = tmp_path / "recording"
    (source / "rgb").mkdir(parents=True)
    (source / "labels").mkdir()
    image = np.zeros((10, 10, 3), dtype=np.uint8)
    cv2.imwrite(str(source / "rgb" / "frame_000000.png"), image)
    cv2.imwrite(str(source / "labels" / "frame_000000.png"), image[:, :, 0])
    (source / "manifest.jsonl").write_text(
        json.dumps({"frame": "frame_000000", "label_source": "colored_map_fallback"}) + "\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="collapses class IDs"):
        convert(source, tmp_path / "dataset", "train", {1: "chair"})


def test_parse_classes_rejects_duplicates():
    with pytest.raises(ValueError, match="unique and positive"):
        parse_classes(["1:chair", "1:couch"])
