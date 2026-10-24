"""Prepared YOLOE weights must start without a text encoder or network."""
import sys
from types import SimpleNamespace

import pytest
import rclpy

from robis_vision.open_vocab_detector import OpenVocabularyDetector


@pytest.mark.parametrize('prepared', [True, False])
def test_yoloe_uses_correct_loader_and_reuses_prepared_prompts(monkeypatch, prepared):
    calls = []

    class Model:
        names = {0: 'door', 1: 'doorway'} if prepared else {0: 'person'}

        def get_text_pe(self, prompts):
            assert not prepared, 'Prepared weights must not load a text encoder'
            calls.append(('encode', prompts))
            return 'embeddings'

        def set_classes(self, prompts, embeddings):
            calls.append(('set_classes', prompts, embeddings))

    def yoloe(path):
        calls.append(('YOLOE', path))
        return Model()

    def wrong_backend(path):
        raise AssertionError('YOLOE requires its own loader')

    monkeypatch.setitem(sys.modules, 'ultralytics',
                        SimpleNamespace(YOLOE=yoloe, YOLOWorld=wrong_backend))
    rclpy.init()
    node = None
    try:
        node = OpenVocabularyDetector()
        assert calls[0] == ('YOLOE', 'yoloe-26s-seg.pt')
        assert len(calls) == (1 if prepared else 3)
        if not prepared:
            assert calls[1] == ('encode', ['door', 'doorway'])
            assert calls[2] == ('set_classes', ['door', 'doorway'], 'embeddings')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
