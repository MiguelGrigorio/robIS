from robis_semantic_navigation.observations import ObservationCache


def test_cache_keeps_independent_backends_until_expiry():
    cache = ObservationCache()
    chair = object()
    red_chair = object()
    first = cache.update({"red chair": [red_chair]}, received_at=10.0)
    cache.update({"chair": [chair]}, received_at=10.1)

    assert cache.snapshot("red chair", 1.0, now=10.2) == (first, [red_chair])
    assert cache.snapshot("chair", 1.0, now=10.2)[1] == [chair]
    assert cache.snapshot("red chair", 1.0, now=11.1) is None


def test_empty_frames_report_liveness_without_reusing_a_sequence():
    cache = ObservationCache()
    cache.update({}, received_at=4.0)

    assert cache.is_alive(0.5, now=4.4)
    assert not cache.is_alive(0.5, now=4.6)
    assert cache.snapshot("chair", 1.0, now=4.1) is None
