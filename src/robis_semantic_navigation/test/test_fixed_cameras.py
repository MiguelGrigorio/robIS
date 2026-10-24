"""Known rays, camera overlap, moving objects and freshness without a robot."""
import json

import numpy as np
import pytest
import yaml

from robis_semantic_navigation.fixed_cameras import FixedCameraMap
from robis_semantic_navigation.fixed_camera_replay import main as replay


def configuration(tmp_path):
    k = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
    for camera_id, x in [('1', 0.), ('2', 1.)]:
        transform = np.diag([1., -1., -1., 1.])
        transform[:3, 3] = [x, 0., 3.]
        np.savez(tmp_path / (camera_id + '.npz'), cam_id=int(camera_id),
                 width=640, height=480, K=k, nK=k, dist=np.zeros(14),
                 rt_inv=transform, rt=np.linalg.inv(transform))
    config = {'environment_id': 'fixture', 'map_checksum': 'a' * 64,
              'map_from_world': {'x': 0., 'y': 0., 'yaw': 0.},
              'planes': {'chair': {'z_m': 0.5, 'height_uncertainty_m': 0.05}},
              'cameras': [{'id': i, 'calibration': i + '.npz', 'image_space': 'raw'}
                          for i in ['1', '2']]}
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump(config))
    return path


def event(camera='1', stamp=10., index=0, x=0.3):
    # Independent analytic pinhole projection: camera looks down from z=3,
    # reference plane is z=.5, so optical depth is exactly 2.5 metres.
    camera_x = 0. if camera == '1' else 1.
    u, v = 320. + (x - camera_x) * 500 / 2.5, 240. - 0.1 * 500 / 2.5
    return {'camera_id': camera, 'stamp': stamp, 'frame_index': index,
            'width': 640, 'height': 480,
            'detections': [{'label': 'chair', 'score': 0.9,
                            'bbox': [u - 10, v - 10, u + 10, v + 10]}]}


def test_two_cameras_create_one_confirmed_track(tmp_path):
    engine = FixedCameraMap(configuration(tmp_path))
    assert not engine.ingest(event())['objects']
    result = engine.ingest(event('2'))['objects']
    assert len(result) == 1
    assert result[0]['position'] == pytest.approx([0.3, 0.1, 0.5])
    assert result[0]['cameras'] == ['1', '2']


def test_motion_replaces_old_position_and_expiry_removes_target(tmp_path):
    engine = FixedCameraMap(configuration(tmp_path))
    engine.ingest(event())
    first = engine.ingest(event('2'))['objects'][0]
    moved = engine.ingest(event(stamp=11., index=1, x=0.7))['objects'][0]
    assert moved['id'] == first['id']
    assert moved['position'][0] == pytest.approx(0.7)
    empty = event(stamp=12., index=2)
    empty['detections'] = []
    assert engine.ingest(empty)['objects'][0]['last_seen'] == 11.
    assert not engine.snapshot(14.)['objects']


def test_close_same_class_objects_are_not_merged_within_one_frame(tmp_path):
    engine = FixedCameraMap(configuration(tmp_path))
    frame = event()
    frame['detections'] += event(x=0.6)['detections']
    engine.ingest(frame)
    other = event('2')
    other['detections'] += event('2', x=0.6)['detections']
    result = engine.ingest(other)['objects']
    assert len(result) == 2 and len({item['id'] for item in result}) == 2


@pytest.mark.parametrize('change', ['duplicate', 'older', 'resize', 'future'])
def test_bad_frames_do_not_refresh_tracks(tmp_path, change):
    engine = FixedCameraMap(configuration(tmp_path))
    engine.ingest(event())
    before = engine.checkpoint()
    bad = event()
    if change == 'older':
        bad.update(frame_index=1, stamp=9.)
    elif change == 'resize':
        bad['width'] = 320
    elif change == 'future':
        bad.update(frame_index=1, stamp=50.)
    with pytest.raises(ValueError):
        engine.ingest(bad, now=10.)
    assert engine.checkpoint() == before


def test_unknown_class_or_uncertain_height_never_yields_a_position(tmp_path):
    path = configuration(tmp_path)
    config = yaml.safe_load(path.read_text())
    config['planes']['chair']['height_uncertainty_m'] = 10.
    path.write_text(yaml.safe_dump(config))
    engine = FixedCameraMap(path)
    frame = event()
    frame['detections'].append({'label': 'person'})
    assert not engine.ingest(frame)['objects']
    assert sum(engine.rejections.values()) == 2


def test_alignment_is_explicit_and_applied_after_camera_extrinsics(tmp_path):
    path = configuration(tmp_path)
    config = yaml.safe_load(path.read_text())
    config['map_from_world'] = {'x': 2., 'y': -1., 'yaw': np.pi / 2}
    path.write_text(yaml.safe_dump(config))
    engine = FixedCameraMap(path)
    engine.ingest(event())
    point = engine.ingest(event('2'))['objects'][0]['position']
    assert point == pytest.approx([1.9, -0.7, 0.5])
    config['map_from_world']['x'] = None
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='explicit'):
        FixedCameraMap(path)


def test_restart_keeps_ids_and_rejects_changed_calibration(tmp_path):
    path = configuration(tmp_path)
    engine = FixedCameraMap(path)
    engine.ingest(event())
    first = engine.ingest(event('2'))['objects'][0]
    saved = json.loads(json.dumps(engine.checkpoint()))
    restored = FixedCameraMap(path)
    restored.restore(saved)
    assert restored.ingest(event(stamp=11., index=1))['objects'][0]['id'] == first['id']
    saved['fingerprint'] = 'changed'
    with pytest.raises(ValueError, match='fingerprint'):
        restored.restore(saved)


def test_recording_replay_distinguishes_detections_from_image_inference(tmp_path):
    path = configuration(tmp_path)
    recording = tmp_path / 'events.jsonl'
    recording.write_text('\n'.join(json.dumps(e) for e in [event(), event('2')]))
    output = tmp_path / 'output.jsonl'
    replay(['--config', str(path), '--input', str(recording), '--output', str(output),
            '--detections-only'])
    records = [json.loads(line) for line in output.read_text().splitlines()]
    assert records[0]['detector'] == 'recorded_detections'
    assert len(records[2]['objects']) == 1 and not records[3]['objects']
    before = output.read_bytes()
    with pytest.raises(SystemExit):
        replay(['--config', str(path), '--input', str(recording), '--output', str(output)])
    assert output.read_bytes() == before


def test_raw_projection_preserves_thin_prism_coefficients_beyond_first_eight(tmp_path):
    path = configuration(tmp_path)
    filename = tmp_path / '1.npz'
    with np.load(filename) as source:
        data = {name: source[name] for name in source.files}
    data['dist'][0], data['dist'][8] = 0.2, 0.02
    np.savez(filename, **data)
    camera = FixedCameraMap(path).cameras['1']
    # x=.12,y=-.04,r²=.016: k1 contributes .0032, thin prism s1 contributes .00032.
    point = camera.intersect([380.352, 219.936], 0.5, 12.)
    assert point == pytest.approx([0.3, 0.1, 0.5], abs=1e-7)
    assert camera.distortion.size == 14


def test_repeated_single_camera_does_not_replace_required_multiview_support(tmp_path):
    path = configuration(tmp_path)
    config = yaml.safe_load(path.read_text())
    config['minimum_cameras'] = 2
    path.write_text(yaml.safe_dump(config))
    engine = FixedCameraMap(path)
    engine.ingest(event())
    assert not engine.ingest(event(stamp=10.1, index=1))['objects']
    assert len(engine.ingest(event('2', stamp=10.1))['objects']) == 1


def test_tentative_duplicate_cannot_steal_an_established_moving_identity(tmp_path):
    path = configuration(tmp_path)
    config = yaml.safe_load(path.read_text())
    config['minimum_cameras'] = 2
    path.write_text(yaml.safe_dump(config))
    engine = FixedCameraMap(path)
    engine.ingest(event())
    first = engine.ingest(event('2'))['objects'][0]
    duplicate = event(stamp=10.5, index=1, x=.4)
    duplicate['detections'] += event(x=.7)['detections']
    engine.ingest(duplicate)
    # Camera 2's estimate lies closer to the tentative duplicate. Prefer the
    # established identity, while retaining the same geometric distance gate.
    output = engine.ingest(event('2', stamp=10.5, index=1, x=.6))['objects']
    assert len(output) == 1 and output[0]['id'] == first['id']
    assert output[0]['position'][0] == pytest.approx(.5)
