"""Moving CCTV identities cannot leave an obsolete Nav2 destination active."""
import asyncio
from copy import deepcopy
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
import pytest
import rclpy
from rclpy.task import Future

from robis_interfaces.action import GotoObject
from robis_semantic_navigation.live_target import LiveTarget, TargetLost, TargetMoved
from robis_semantic_navigation.server import SemanticNavigationServer


def test_movement_accumulates_from_planned_position_and_never_changes_identity():
    initial = {'id': 'selected', 'pose': {'x': 0., 'y': 0.}}
    target = LiveTarget(initial, .35)
    for x in [.1, .2, .3]:
        current = deepcopy(initial)
        current['pose']['x'] = x
        assert target.check(current) == current
    current['pose']['x'] = .36
    with pytest.raises(TargetMoved):
        target.check(current)
    with pytest.raises(TargetLost):
        target.check(None)
    current['id'] = 'another-chair'
    with pytest.raises(TargetLost):
        target.check(current)


@pytest.fixture
def server(tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'semantic_map:={tmp_path}/objects.yaml'])
    node = SemanticNavigationServer()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def seed(server, x=3., identity='ceiling:track'):
    return server.store.upsert_external(
        'lab', 'chair', identity, {'x': x, 'y': 0.}, .9,
        server.get_clock().now().nanoseconds, time.time() + 3, 'a' * 64)


def ready(value):
    future = Future()
    future.set_result(value)
    return future


@pytest.mark.parametrize('lost', [False, True])
def test_active_navigation_is_canceled_before_replan_or_expiry_result(server, lost, monkeypatch):
    first = seed(server)
    events = []
    server._robot_xy = lambda: (0., 0.)
    remote = ready(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))

    class PendingResult:
        finished = False

        def done(self):
            if not self.finished and not events:
                if lost:
                    server.store.mark_stale('lab', 'chair', first['id'])
                    seed(server, x=3.1, identity='ceiling:other-chair')
                else:
                    seed(server, x=3.5)
            return self.finished

    pending = PendingResult()

    def cancel():
        events.append('old-goal-canceled')
        pending.finished = True
        return remote

    goal = SimpleNamespace(accepted=True, get_result_async=lambda: pending, cancel_goal_async=cancel)
    server.nav_client = SimpleNamespace(send_goal_async=lambda _: ready(goal))

    async def select(x, y, handle, deadline):
        if events:
            events.append(('replan', x, y))
            handle.is_cancel_requested = True
        return server._pose(x - 1.4, y, 0.)

    server._select_approach_pose = select
    monkeypatch.setattr(time, 'sleep', lambda _: None)
    handle = SimpleNamespace(is_cancel_requested=False, publish_feedback=lambda _: None,
                             abort=lambda: events.append('aborted'), canceled=lambda: events.append('canceled'))
    started = time.monotonic()
    result = asyncio.run(server._visit_live_memory(
        handle, 'lab', 'chair', first, started, started + 5))
    assert events[0] == 'old-goal-canceled'
    if lost:
        assert events == ['old-goal-canceled', 'aborted']
        assert result.error_code == GotoObject.Result.NOT_FOUND
    else:
        assert events[1] == ('replan', 3.5, 0.)
        assert result.error_code == GotoObject.Result.CANCELLED
    assert server.active_nav_goal is None and server._live_target is None


def test_onboard_confirmation_keeps_selected_external_id_instead_of_nearest_neighbor(server):
    selected = seed(server, x=2.)
    other = seed(server, x=2.2, identity='ceiling:other')
    server._live_target = LiveTarget(selected, .35)
    server._live_environment, server._live_label = 'lab', 'chair'
    server._robot_xy = lambda: (.8, 0.)
    handle = SimpleNamespace(is_cancel_requested=False, publish_feedback=lambda _: None,
                             succeed=lambda: None, abort=lambda: None)
    result = asyncio.run(server._finish(handle, 'lab', 'chair', (.9, Point(x=2.19)), time.monotonic()))
    assert result.success
    entries = {item['id']: item for item in server.store.instances('lab', 'chair')}
    assert entries[selected['id']]['onboard_confirmations'] == 1
    assert entries[selected['id']]['pose']['x'] == 2.19
    assert entries[other['id']].get('onboard_confirmations', 0) == 0


def test_expired_target_cannot_be_persisted_as_success(server):
    selected = seed(server, x=2.)
    server._live_target = LiveTarget(selected, .35)
    server._live_environment, server._live_label = 'lab', 'chair'
    server.store.mark_stale('lab', 'chair', selected['id'])
    server._robot_xy = lambda: (.6, 0.)
    handle = SimpleNamespace(is_cancel_requested=False)
    with pytest.raises(TargetLost):
        asyncio.run(server._finish(handle, 'lab', 'chair', (.9, Point(x=2.)), time.monotonic()))


def test_continually_moving_target_has_a_finite_replan_budget(server):
    server.set_parameters([rclpy.parameter.Parameter('target_max_replans', value=2)])
    first = seed(server)
    attempts = []

    async def moving(handle, environment, label, known, started, deadline):
        attempts.append(known['id'])
        raise TargetMoved(seed(server, x=known['pose']['x'] + .4))

    server._visit_memory = moving
    handle = SimpleNamespace(is_cancel_requested=False, publish_feedback=lambda _: None, abort=lambda: None)
    started = time.monotonic()
    result = asyncio.run(server._visit_live_memory(handle, 'lab', 'chair', first, started, started + 5))
    assert attempts == [first['id']] * 3
    assert result.error_code == GotoObject.Result.NOT_FOUND
    assert server._live_target is None
