"""A real SensorDataQoS laser must reach the semantic localizer."""

import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from robis_semantic_navigation.object_localizer import ObjectLocalizer


def test_best_effort_lidar_reaches_localizer():
    rclpy.init(args=[])
    source = Node('physical_lidar_contract_test')
    localizer = ObjectLocalizer()
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(localizer)
    publisher = source.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and localizer.scan is None:
            message = LaserScan()
            message.header.stamp = source.get_clock().now().to_msg()
            message.header.frame_id = 'lidar_link'
            message.ranges = [1.25]
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.02)
        assert localizer.scan is not None
        assert list(localizer.scan.ranges) == [1.25]
        assert localizer.scan_received_at is not None
    finally:
        executor.shutdown()
        source.destroy_node()
        localizer.destroy_node()
        rclpy.shutdown()
