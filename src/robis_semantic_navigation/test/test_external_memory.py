"""Environment observations seed memory without bypassing robot reconfirmation."""
import asyncio
import copy
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose

from robis_interfaces.action import GotoObject
from robis_interfaces.msg import ExternalObject, ExternalObjects
from robis_semantic_navigation.server import SemanticNavigationServer
from robis_semantic_navigation.store import SemanticStore


@pytest.fixture
def external_server(tmp_path):
    rclpy.init(args=['--ros-args', '-p', 'enable_external_memory:=true',
                     '-p', 'external_environment_id:=labsea', '-p', 'map_checksum:=' + 'a' * 64,
                     '-p', 'semantic_map:=' + str(tmp_path / 'memory.yaml')])
    node = SemanticNavigationServer()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def message(server, x=1.4):
    output = ExternalObjects()
    output.header.frame_id = 'map'
    output.header.stamp = server.get_clock().now().to_msg()
    output.source_id = 'ceiling'
    output.environment_id = 'labsea'
    output.map_checksum = 'a' * 64
    output.validity_sec = 3.
    detection = Detection3D()
    detection.header = copy.deepcopy(output.header)
    detection.id = 'chair-track'
    detection.bbox.center.position.x = x
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = 'chair'
    result.hypothesis.score = 0.9
    detection.results.append(result)
    observation = ExternalObject()
    observation.detection = detection
    observation.revision = 1
    output.objects.append(observation)
    return output


def test_real_ros_transport_updates_one_persistent_instance(external_server):
    server = external_server
    sender = Node('external_camera_contract_test')
    publisher = sender.create_publisher(ExternalObjects, '/semantic_navigation/external_objects', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(sender)

    def receive(output, predicate):
        deadline = time.monotonic() + 4.
        while time.monotonic() < deadline:
            publisher.publish(output)
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError('external observation did not reach memory')

    try:
        receive(message(server), lambda: bool(server.store.instances('labsea', 'chair')))
        first = server.store.select('labsea', 'chair')
        receive(message(server, 1.8), lambda: server.store.select('labsea', 'chair')['pose']['x'] == 1.8)
        restored = SemanticStore(server.store.path).instances('labsea', 'chair')
        assert len(restored) == 1 and restored[0]['id'] == first['id']
        assert restored[0]['pose']['x'] == 1.8
    finally:
        executor.shutdown()
        sender.destroy_node()


@pytest.mark.parametrize('invalid', ['environment', 'checksum', 'frame', 'expired', 'future', 'nan', 'sensitivity'])
def test_invalid_external_messages_do_not_create_landmarks(external_server, invalid):
    server = external_server
    output = message(server)
    if invalid == 'environment':
        output.environment_id = 'other-room'
    elif invalid == 'checksum':
        output.map_checksum = 'b' * 64
    elif invalid == 'frame':
        output.header.frame_id = 'world'
    elif invalid == 'expired':
        output.objects[0].detection.header.stamp.sec -= 5
    elif invalid == 'future':
        output.objects[0].detection.header.stamp.sec += 5
    elif invalid == 'sensitivity':
        output.objects[0].projection_sensitivity_m = 99.
    else:
        output.objects[0].detection.bbox.center.position.x = float('nan')
    server._external_objects_callback(output)
    assert not server.store.instances('labsea', 'chair')


def test_repeated_snapshot_never_renews_the_original_measurement(external_server):
    server = external_server
    output = message(server)
    server._external_objects_callback(output)
    first = server.store.select('labsea', 'chair')
    output.header.stamp = server.get_clock().now().to_msg()
    server._external_objects_callback(output)
    repeated = server.store.select('labsea', 'chair')
    assert repeated['observations'] == 1
    assert repeated['expires_at_unix'] == first['expires_at_unix']


def test_external_landmark_cannot_count_as_an_onboard_observation(external_server):
    server = external_server
    server._external_objects_callback(message(server))
    assert server.store.select('labsea', 'chair')
    assert server.observations.snapshot('chair', 1.) is None
    assert server._matching_observation('chair') is None


def test_external_landmark_selects_memory_before_any_search(external_server):
    server = external_server
    server._external_objects_callback(message(server))
    states = []
    handle = SimpleNamespace(
        request=SimpleNamespace(timeout_sec=5., environment_id='labsea', query='chair'),
        is_cancel_requested=False, canceled=lambda: None, abort=lambda: None)

    def feedback(value):
        states.append(value.state)
        # Stop at the decision boundary; no navigation goal is sent in this test.
        if value.state == GotoObject.Feedback.NAVIGATING_TO_MEMORY:
            handle.is_cancel_requested = True

    async def ready(*args):
        return True

    handle.publish_feedback = feedback
    server._wait_servers = ready
    server._wait_perception = ready
    server._robot_xy = lambda: (0., 0.)
    result = asyncio.run(server._execute(handle))
    assert GotoObject.Feedback.NAVIGATING_TO_MEMORY in states
    assert GotoObject.Feedback.SEARCHING not in states
    assert result.error_code == GotoObject.Result.CANCELLED
