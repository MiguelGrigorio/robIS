import asyncio
import time
from types import SimpleNamespace

from geometry_msgs.msg import Point
from action_msgs.msg import GoalStatus
import pytest
import rclpy
from rclpy.task import Future
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from robis_semantic_navigation.server import SemanticNavigationServer


@pytest.fixture
def server(tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'semantic_map:={tmp_path}/objects.yaml'])
    node = SemanticNavigationServer()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def observation(frame='map', x=2.0, score=1.0):
    message = Detection3DArray()
    message.header.frame_id = frame
    detection = Detection3D()
    detection.bbox.center.position.x = x
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = 'chair'
    result.hypothesis.score = score
    detection.results = [result]
    message.detections = [detection]
    return message


@pytest.mark.parametrize('message', [
    observation(frame='odom'), observation(x=float('nan')),
    observation(score=float('inf')), observation(score=1.2),
])
def test_invalid_coordinate_or_confidence_cannot_enter_memory(server, message):
    server._objects_callback(message)
    assert server.observations.snapshot('chair', 1.0) is None


def test_map_observation_is_accepted(server):
    server._objects_callback(observation())
    assert server.observations.snapshot('chair', 1.0) is not None


@pytest.mark.parametrize('robot_x, success', [(0.6, True), (-1.0, False)])
def test_success_requires_final_standoff_before_persisting(server, robot_x, success):
    server._robot_xy = lambda: (robot_x, 0.0)
    calls = []
    handle = SimpleNamespace(
        is_cancel_requested=False,
        succeed=lambda: calls.append('succeeded'), abort=lambda: calls.append('aborted'),
        publish_feedback=lambda _: None)
    point = Point(x=2.0)
    result = asyncio.run(server._finish(handle, 'lab', 'chair', (1.0, point), 0.0))
    assert result.success is success
    assert bool(server.store.instances('lab', 'chair')) is success
    assert calls == ['succeeded' if success else 'aborted']


@pytest.mark.parametrize('accepted', [True, False])
def test_delayed_acceptance_cancels_remote_goal_after_parent_timeout(server, accepted):
    pending = Future()
    calls = []
    server._cancel_late_goal(pending)
    assert not pending.cancelled()
    pending.set_result(SimpleNamespace(
        accepted=accepted,
        cancel_goal_async=lambda: calls.append('remote_cancel')))
    assert calls == (['remote_cancel'] if accepted else [])


def test_expired_mission_does_not_start_new_motion(server):
    handle = SimpleNamespace(is_cancel_requested=False)
    pose = server._pose(0.0, 0.0, 0.0)
    assert asyncio.run(server._compute_path(pose, handle, 0.0)) is None
    assert asyncio.run(server._navigate(pose, handle, 0.0)) is False
    assert asyncio.run(server._navigate_until_seen(pose, 'chair', handle, 0.0)) == (False, None)
    assert asyncio.run(server._spin_segment(1.0, handle, 0.0)) is False
    assert asyncio.run(server._spin_until_seen(1.0, 'chair', handle, 0.0)) == (False, None)


@pytest.mark.parametrize('keeps_shifting', [False, True])
def test_refined_position_gets_one_correction_and_never_bypasses_standoff(server, keeps_shifting):
    position = [0.0, 0.0]
    planned = []
    confirmations = iter([2.7, 3.4 if keeps_shifting else 2.7])
    server._robot_xy = lambda: tuple(position)

    async def select(x, y, handle, deadline):
        planned.append((x, y))
        return (x - 1.4, y)

    async def navigate(pose, handle, deadline):
        position[:] = pose
        return True

    async def confirm(label, handle, deadline):
        return (0.9, Point(x=next(confirmations)))

    server._select_approach_pose = select
    server._navigate = navigate
    server._reconfirm_with_scan = confirm
    calls = []
    handle = SimpleNamespace(is_cancel_requested=False, publish_feedback=lambda _: None,
                             succeed=lambda: calls.append('success'), abort=lambda: calls.append('abort'))
    now = time.monotonic()
    result = asyncio.run(server._approach_and_finish(
        handle, 'lab', 'chair', (0.8, Point(x=2.0)), now, now + 10))
    assert len(planned) == 2
    assert result.success is (not keeps_shifting)
    assert bool(server.store.instances('lab', 'chair')) is (not keeps_shifting)
    assert calls == (['abort'] if keeps_shifting else ['success'])


@pytest.mark.parametrize('fresh_at_arrival', [False, True])
def test_route_does_not_return_an_old_high_confidence_observation(server, monkeypatch, fresh_at_arrival):
    class NavigationResult:
        checks = 0

        def done(self):
            self.checks += 1
            return self.checks > 4

        def result(self):
            return SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)

    goal = SimpleNamespace(accepted=True, get_result_async=lambda: NavigationResult())
    accepted = Future()
    accepted.set_result(goal)
    server.nav_client = SimpleNamespace(send_goal_async=lambda _: accepted)
    old = (0.99, Point(x=2.0))
    fresh = (0.6, Point(x=2.4))
    snapshots = iter(
        [(i, old) for i in range(4)]
        + ([(5, fresh)] if fresh_at_arrival else [None]))
    server._matching_observation = lambda _: next(snapshots)
    monkeypatch.setattr(time, 'sleep', lambda _: None)
    result = asyncio.run(server._navigate_until_seen(
        server._pose(0.0, 0.0, 0.0), 'chair',
        SimpleNamespace(is_cancel_requested=False), time.monotonic() + 10))
    assert result == (True, fresh if fresh_at_arrival else None)
