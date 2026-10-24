"""The live queue must not combine different captures or accumulate stale work."""
from robis_semantic_navigation.fixed_camera_stream import SynchronizedImages


def test_only_complete_captures_are_processed_and_duplicates_are_ignored():
    queue = SynchronizedImages(['1', '2'])
    queue.add('1', 10, 'one')
    queue.add('2', 11, 'different time')
    assert queue.take_latest() is None
    queue.add('2', 10, 'two')
    assert queue.take_latest() == (10, {'1': 'one', '2': 'two'})
    queue.add('1', 10, 'duplicate')
    assert queue.take_latest() is None


def test_newest_complete_capture_wins_without_consuming_a_future_partial_capture():
    queue = SynchronizedImages(['1', '2'])
    for stamp in [10, 20]:
        for camera in ['1', '2']:
            queue.add(camera, stamp, camera)
    queue.add('1', 30, 'future')
    assert queue.take_latest()[0] == 20
    assert queue.dropped == 1 and set(queue.pending) == {30}


def test_missing_camera_has_bounded_memory_and_cannot_publish_an_incomplete_set():
    queue = SynchronizedImages(['1', '2'], maximum_batches=2)
    for stamp in range(20):
        queue.add('1', stamp, 'image')
    assert len(queue.pending) == 2 and queue.dropped == 18
    assert queue.take_latest() is None
