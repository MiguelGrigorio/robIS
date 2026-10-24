import pytest

from robis_semantic_navigation.store import SemanticStore


def test_multiple_instances_and_selection(tmp_path):
    store = SemanticStore(tmp_path / "semantic.yaml")
    first = store.upsert("lab", "chair", {"x": 1.0, "y": 0.0, "yaw": 0.0}, 0.7)
    second = store.upsert("lab", "chair", {"x": 4.0, "y": 0.0, "yaw": 0.0}, 0.9)
    assert first["id"] != second["id"]
    assert len(store.instances("lab", "chair")) == 2
    assert store.select("lab", "chair", "nearest", (0.0, 0.0))["id"] == first["id"]
    assert store.select("lab", "chair", "highest_confidence")["id"] == second["id"]


def test_upsert_merges_and_survives_restart(tmp_path):
    filename = tmp_path / "semantic.yaml"
    store = SemanticStore(filename)
    item = store.upsert("lab", "chair", {"x": 1.0, "y": 2.0, "yaw": 0.0}, 0.6)
    merged = store.upsert("lab", "chair", {"x": 1.1, "y": 2.0, "yaw": 0.1}, 0.8)
    assert item["id"] == merged["id"]
    assert SemanticStore(filename).instances("lab", "chair")[0]["observations"] == 2


def test_backup_recovers_corrupt_primary(tmp_path):
    filename = tmp_path / "semantic.yaml"
    store = SemanticStore(filename)
    store.upsert("lab", "chair", {"x": 1.0, "y": 2.0, "yaw": 0.0}, 0.6)
    store.upsert("lab", "table", {"x": 3.0, "y": 2.0, "yaw": 0.0}, 0.7)
    filename.write_text("not: [valid", encoding="utf-8")
    recovered = SemanticStore(filename)
    assert recovered.instances("lab", "chair")


def test_checksum_mismatch_is_rejected(tmp_path):
    store = SemanticStore(tmp_path / "semantic.yaml")
    store.upsert(
        "lab", "chair", {"x": 1.0, "y": 2.0, "yaw": 0.0}, 0.8,
        map_checksum="abc",
    )
    with pytest.raises(ValueError, match="checksum"):
        store.environment("lab", "def")
    with pytest.raises(ValueError, match="checksum"):
        store.select("lab", "chair", map_checksum="def")
    assert store.select("lab", "chair", map_checksum="abc")


def test_verified_stale_instance_is_not_selected(tmp_path):
    store = SemanticStore(tmp_path / "semantic.yaml")
    item = store.upsert(
        "lab", "chair", {"x": 1.0, "y": 2.0, "yaw": 0.0}, 1.0
    )
    store.mark_stale("lab", "chair", item["id"])

    assert store.select("lab", "chair") is None
    assert store.instances("lab", "chair", valid_only=False)[0]["valid"] is False


def test_external_pose_moves_without_averaging_and_expires_after_restart(tmp_path, monkeypatch):
    from robis_semantic_navigation import store as module
    monkeypatch.setattr(module.time, 'time', lambda: 100.)
    path = tmp_path / 'semantic.yaml'
    store = SemanticStore(path)
    first = store.upsert_external('lab', 'chair', 'source:track',
                                  {'x': 1., 'y': 0.}, 0.8, 100, 103., 'a' * 64)
    moved = store.upsert_external('lab', 'chair', 'source:track',
                                  {'x': 2., 'y': 0.}, 0.7, 101, 104., 'a' * 64)
    assert moved['id'] == first['id']
    assert moved['pose']['x'] == 2. and moved['confidence'] == 0.7
    assert store.upsert_external('lab', 'chair', 'source:track',
                                 {'x': 9., 'y': 0.}, 1., 101, 200., 'a' * 64) is None
    assert store.instances('lab', 'chair')[0]['expires_at_unix'] == 104.
    monkeypatch.setattr(module.time, 'time', lambda: 105.)
    assert SemanticStore(path).select('lab', 'chair') is None
    assert len(SemanticStore(path).instances('lab', 'chair', valid_only=False)) == 1


def test_onboard_confirmation_refreshes_only_the_volatile_lifetime(tmp_path, monkeypatch):
    from robis_semantic_navigation import store as module
    monkeypatch.setattr(module.time, 'time', lambda: 100.)
    store = SemanticStore(tmp_path / 'memory.yaml')
    first = store.upsert_external('lab', 'chair', 'source:track',
                                  {'x': 1., 'y': 0.}, 0.8, 100, 103., 'a' * 64)
    monkeypatch.setattr(module.time, 'time', lambda: 104.)
    fresh = store.upsert('lab', 'chair', {'x': 1., 'y': 0.}, 0.9)
    assert fresh['id'] == first['id']
    assert store.select('lab', 'chair')['expires_at_unix'] == 107.
    assert fresh['onboard_confirmations'] == 1
    assert fresh['last_onboard_confirmed']


def test_external_observation_refuses_wrong_map_and_expired_sample(tmp_path):
    import time
    store = SemanticStore(tmp_path / 'memory.yaml')
    store.environment('lab', 'a' * 64)
    with pytest.raises(ValueError, match='checksum'):
        store.upsert_external('lab', 'chair', 'id', {'x': 0., 'y': 0.},
                              0.9, 1, time.time() + 3, 'b' * 64)
    with pytest.raises(ValueError, match='expired'):
        store.upsert_external('lab', 'chair', 'id', {'x': 0., 'y': 0.},
                              0.9, 1, time.time() - 3, 'a' * 64)
    assert store.select('lab', 'chair') is None


def test_ceiling_updates_preserve_but_never_increment_onboard_confirmation(tmp_path):
    import time
    store = SemanticStore(tmp_path / 'memory.yaml')
    first = store.upsert_external('lab', 'chair', 'source:track', {'x': 1., 'y': 0.},
                                  0.8, 100, time.time() + 3, 'a' * 64)
    assert first.get('onboard_confirmations', 0) == 0
    onboard = store.upsert('lab', 'chair', {'x': 1.1, 'y': 0.}, 0.9)
    updated = store.upsert_external('lab', 'chair', 'source:track', {'x': 1.2, 'y': 0.},
                                    0.8, 200, time.time() + 3, 'a' * 64)
    assert updated['id'] == first['id'] and updated['onboard_confirmations'] == 1
    assert updated['last_onboard_confirmed'] == onboard['last_onboard_confirmed']
    assert SemanticStore(store.path).select('lab', 'chair')['onboard_confirmations'] == 1


def test_failed_onboard_confirmation_quarantines_only_that_external_track(tmp_path):
    import time
    store = SemanticStore(tmp_path / 'memory.yaml')
    first = store.upsert_external('lab', 'bed', 'camera:bad', {'x': 1., 'y': 0.},
                                  .9, 100, time.time() + 3, 'a' * 64)
    store.mark_stale('lab', 'bed', first['id'])
    store = SemanticStore(store.path)
    assert store.upsert_external('lab', 'bed', 'camera:bad', {'x': 1., 'y': 0.},
                                 .99, 200, time.time() + 3, 'a' * 64) is None
    assert not store.instances('lab', 'bed')
    different = store.upsert_external('lab', 'bed', 'camera:new', {'x': 3., 'y': 0.},
                                      .9, 200, time.time() + 3, 'a' * 64)
    assert different and len(store.instances('lab', 'bed')) == 1
    board = store.upsert('lab', 'bed', {'x': 1., 'y': 0.}, .95)
    assert board['id'] == first['id'] and not board.get('onboard_rejected', False)


def test_simultaneous_camera_revision_refines_pose_without_extending_freshness(tmp_path, monkeypatch):
    from robis_semantic_navigation import store as module
    monkeypatch.setattr(module.time, 'time', lambda: 100.)
    store = SemanticStore(tmp_path / 'memory.yaml')
    first = store.upsert_external('lab', 'chair', 'track', {'x': 1., 'y': 0.},
                                  0.8, 100, 103., 'a' * 64, revision=2)
    refined = store.upsert_external('lab', 'chair', 'track', {'x': 1.1, 'y': 0.},
                                    0.9, 100, 105., 'a' * 64, revision=3)
    assert refined['id'] == first['id']
    assert refined['pose']['x'] == 1.1 and refined['expires_at_unix'] == 103.
    assert store.upsert_external('lab', 'chair', 'track', {'x': 9., 'y': 0.},
                                 1., 100, 200., 'a' * 64, revision=3) is None


def test_ros_clock_expiry_cannot_be_bypassed_by_a_faster_simulation(tmp_path):
    import time
    store = SemanticStore(tmp_path / 'memory.yaml')
    store.upsert_external('lab', 'chair', 'track', {'x': 1., 'y': 0.},
                          0.8, 1000000000, time.time() + 3, 'a' * 64, validity_sec=3.)
    assert store.select('lab', 'chair', ros_now_ns=2000000000)
    assert store.select('lab', 'chair', ros_now_ns=4000000001) is None
