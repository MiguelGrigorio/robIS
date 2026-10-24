"""A broad segmented tabletop keeps its anchor when a long leg enters the bbox."""
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from robis_semantic_navigation.image_anchors import mask_incenter, yolo_detections
from robis_semantic_navigation.fixed_cameras import FixedCameraMap
from test_fixed_cameras import configuration, event


def test_tabletop_anchor_does_not_follow_leg_or_bounding_box_center():
    mask = np.zeros((160, 160), np.uint8)
    cv2.ellipse(mask, (80, 40), (35, 18), 0, 0, 360, 1, -1)
    mask[40:145, 78:82] = 1
    anchor = mask_incenter(mask, [40., 20., 120., 150.])
    assert anchor == pytest.approx([80., 40.], abs=3.)
    assert abs(anchor[1] - 85.) > 30
    assert mask[round(anchor[1]), round(anchor[0])]


@pytest.mark.parametrize('mask', [np.zeros((20, 20)), np.eye(20)])
def test_empty_or_thin_mask_has_no_surface_anchor(mask):
    assert mask_incenter(mask, [0., 0., 20., 20.]) is None


def test_segmentation_config_does_not_silently_fall_back_to_bbox(tmp_path):
    import yaml
    path = configuration(tmp_path)
    value = yaml.safe_load(path.read_text())
    value['planes']['chair']['anchor_method'] = 'mask_incenter'
    path.write_text(yaml.safe_dump(value))
    engine = FixedCameraMap(path)
    with pytest.raises(ValueError, match='segmentation anchor required'):
        engine.project(engine.cameras['1'], event()['detections'][0])


def test_missing_mask_drops_only_classes_requiring_segmentation():
    class Tensor:
        def cpu(self):
            return self

        def tolist(self):
            return [1., 1., 9., 9.]

    boxes = [SimpleNamespace(cls=[index], conf=[.9], xyxy=[Tensor()]) for index in range(2)]
    result = SimpleNamespace(names={0: 'dining table', 1: 'chair'}, boxes=boxes, masks=None)
    output = yolo_detections(result, {'dining table': {'anchor_method': 'mask_incenter'}}, 10, 10)
    assert [item['label'] for item in output] == ['chair']
