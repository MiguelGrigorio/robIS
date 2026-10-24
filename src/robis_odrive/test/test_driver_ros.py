"""ROS messages/services through the simulated adapter; never imports USB API."""

import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from robis_odrive.odrive_node import OdriveNode
from robis_odrive.odrive_simulator import ODriveInterfaceSimulator


class TrackingSimulator(ODriveInterfaceSimulator):
    def __init__(self):
        super().__init__()
        self.calls = []

    def connect(self, **kwargs):
        self.calls.append(('connect', kwargs))
        return super().connect(**kwargs)

    def calibrate(self, **kwargs):
        self.calls.append(('calibrate', kwargs))
        return super().calibrate(**kwargs)

    def engage(self):
        self.calls.append(('engage', {}))
        return super().engage()


class TestDriverRos(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.probe = Node('odrive_test_probe')
        self.executor.add_node(self.probe)
        self.node = None
        self.driver = TrackingSimulator()
        self.messages = {'odom': [], 'raw': [], 'tf': [], 'current': []}
        for name, topic, kind in [
            ('odom', '/test/odom', Odometry),
            ('raw', '/odrive/left/raw_odom/encoder', Float64),
            ('tf', '/tf', TFMessage),
            ('current', '/odrive/left/current', Float64),
        ]:
            self.probe.create_subscription(kind, topic,
                lambda msg, key=name: self.messages[key].append(msg), 10)
        self.publisher = self.probe.create_publisher(Twist, '/test/cmd_vel', 10)

    def tearDown(self):
        if self.node is not None:
            self.node.shutdown()
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        self.executor.remove_node(self.probe)
        self.probe.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()

    def start(self, **settings):
        values = dict(simulation_mode=True, odometry_mode='encoder',
            odom_topic='/test/odom', cmd_vel_topic='/test/cmd_vel',
            odom_frame='test_odom', base_frame='test_base',
            connect_on_startup=True, command_timeout_sec=0.15,
            idle_release_sec=0.2, odom_calc_hz=50)
        values.update(settings)
        self.node = OdriveNode(driver=self.driver,
            parameter_overrides=[Parameter(name, value=value) for name, value in values.items()])
        self.executor.add_node(self.node)
        self.wait(lambda: self.publisher.get_subscription_count() > 0)

    def wait(self, condition, timeout=3):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        self.assertTrue(condition(), 'ROS condition timed out')

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def command(self, linear=0.0, angular=0.0, seconds=0.3):
        deadline = time.monotonic() + seconds
        message = Twist()
        message.linear.x, message.angular.z = linear, angular
        while time.monotonic() < deadline:
            self.publisher.publish(message)
            self.executor.spin_once(timeout_sec=0.01)
            time.sleep(0.01)

    def call(self, name):
        topic = '/reset_odometry' if name == 'reset_odometry' else '/odrive_node/' + name
        client = self.probe.create_client(Trigger, topic)
        self.assertTrue(client.wait_for_service(timeout_sec=2))
        future = client.call_async(Trigger.Request())
        self.wait(future.done)
        result = future.result()
        self.probe.destroy_client(client)
        return result

    def test_velocity_message_reaches_encoder_feedback_odom_and_tf(self):
        self.start()
        self.command(linear=0.1)
        self.wait(lambda: self.messages['odom'] and self.messages['tf'])
        message = self.messages['odom'][-1]
        self.assertGreater(message.pose.pose.position.x, 0.01)
        self.assertAlmostEqual(message.twist.twist.linear.x, 0.1, places=5)
        self.assertEqual((message.header.frame_id, message.child_frame_id), ('test_odom', 'test_base'))
        # Odometry and TF arrive on separate subscriptions. Wait for this
        # odometry sample's transform rather than assuming callback order.
        self.wait(lambda: any(
            transform.header.stamp == message.header.stamp
            for item in self.messages['tf'] for transform in item.transforms))
        transforms = [t for msg in self.messages['tf'] for t in msg.transforms]
        matching = [t for t in transforms if t.header.stamp == message.header.stamp]
        self.assertTrue(matching)
        self.assertAlmostEqual(matching[-1].transform.translation.x, message.pose.pose.position.x)
        self.assertTrue(self.messages['raw'])

    def test_timeout_releases_and_resume_runs_reverse(self):
        self.start()
        self.command(linear=0.1)
        self.wait(lambda: not self.node.controller.engaged)
        self.assertEqual((self.driver.left_axis_vel, self.driver.right_axis_vel), (0, 0))
        self.command(linear=-0.1)
        self.assertTrue(self.node.controller.engaged)
        self.wait(lambda: self.messages['odom'][-1].twist.twist.linear.x < 0)

    def test_disabled_odom_raw_current_and_tf_publish_nothing_but_integrate(self):
        self.start(publish_odom=False, publish_raw_odom=False,
                   publish_current=False, publish_odom_tf=False)
        self.command(linear=0.1)
        self.assertGreater(self.node.controller.odometry.state.x, 0.01)
        self.assertTrue(all(not messages for messages in self.messages.values()))

    def test_tf_can_be_enabled_independently_of_odom(self):
        self.start(publish_odom=False, publish_odom_tf=True)
        self.wait(lambda: bool(self.messages['tf']))
        self.assertFalse(self.messages['odom'])

    def test_startup_flags_and_manual_connection_services(self):
        self.start(connect_on_startup=False, engage_on_startup=False)
        self.spin(0.1)
        self.assertEqual(self.driver.calls, [])
        self.assertTrue(self.call('connect').success)
        self.assertFalse(self.node.controller.engaged)
        self.command()  # zero commands cannot engage
        self.assertFalse(self.node.controller.engaged)
        self.assertTrue(self.call('calibrate').success)
        self.assertTrue(self.call('engage').success)
        self.assertTrue(self.call('release').success)
        self.assertTrue(self.call('disconnect').success)
        self.assertFalse(self.driver.connected)

    def test_reset_service_reports_success_and_no_encoder_jump(self):
        self.start()
        self.command(linear=0.1)
        self.command()
        response = self.call('reset_odometry')
        self.assertTrue(response.success)
        self.assertTrue(response.message)
        self.spin(0.1)
        self.assertAlmostEqual(self.messages['odom'][-1].pose.pose.position.x, 0)

    def test_nan_command_stops_without_refreshing_watchdog(self):
        self.start(command_timeout_sec=2.0)
        command = Twist()
        command.linear.x = 0.1
        self.publisher.publish(command)
        self.wait(lambda: self.driver.left_axis_vel != 0)
        received = self.node.controller.watchdog.last_command_at
        received_invalid = []
        original = self.node.controller.command
        def observe(linear, angular, stamp):
            if linear != linear:
                received_invalid.append(True)
            return original(linear, angular, stamp)
        self.node.controller.command = observe
        command.linear.x = float('nan')
        self.publisher.publish(command)
        self.wait(lambda: bool(received_invalid))
        self.assertEqual(self.node.controller.watchdog.last_command_at, received)
        self.assertEqual((self.driver.left_axis_vel, self.driver.right_axis_vel), (0, 0))

    def test_startup_config_is_read_only_instead_of_silently_ignored(self):
        self.start()
        result = self.node.set_parameters([Parameter('publish_odom', value=False)])[0]
        self.assertFalse(result.successful)
        self.assertTrue(self.node.get_parameter('publish_odom').value)

    def test_feedback_failure_closes_adapter_and_requires_manual_reconnect(self):
        self.start()
        original = self.driver.left_pos
        def disconnected():
            raise RuntimeError('Simulated USB disconnect')
        self.driver.left_pos = disconnected
        self.wait(lambda: not self.node.controller.connected)
        self.spin(0.1)
        self.assertEqual(sum(name == 'connect' for name, _ in self.driver.calls), 1)
        self.driver.left_pos = original
        self.assertTrue(self.call('connect').success)
        self.assertTrue(self.node.controller.connected)

    def test_startup_calibrates_before_engaging_when_requested(self):
        self.start(calibrate_on_startup=True, engage_on_startup=True)
        self.assertEqual([name for name, _ in self.driver.calls][:3], ['connect', 'calibrate', 'engage'])

    def test_unconfirmed_angular_command_stops_without_refreshing_deadline(self):
        self.start(allow_angular_motion=False, command_timeout_sec=2.0)
        self.command(linear=0.1, seconds=0.1)
        received = self.node.controller.watchdog.last_command_at
        self.command(angular=0.1, seconds=0.1)
        self.assertEqual(self.node.controller.watchdog.last_command_at, received)
        self.assertEqual((self.driver.left_axis_vel, self.driver.right_axis_vel), (0, 0))

    def test_positive_ros_yaw_uses_right_hand_convention_when_explicitly_enabled(self):
        self.start(allow_angular_motion=True)
        self.command(angular=0.2)
        self.wait(lambda: self.messages['odom'] and self.messages['odom'][-1].pose.pose.orientation.z > 0)
        self.assertAlmostEqual(self.messages['odom'][-1].twist.twist.angular.z, 0.2, places=5)
        self.assertLess(self.driver.left_axis_vel, -self.driver.right_axis_vel)

    def test_telemetry_guard_fault_disconnects_and_cannot_implicitly_rearm(self):
        self.start()
        self.command(linear=0.1, seconds=0.1)
        def failed_health():
            raise RuntimeError('Simulated overcurrent')
        self.driver.check_health = failed_health
        self.wait(lambda: not self.node.controller.connected)
        self.command(linear=0.1, seconds=0.1)
        self.assertFalse(self.node.controller.engaged)
        self.assertFalse(self.driver.connected)

    def test_reset_service_read_failure_also_disconnects_the_active_session(self):
        self.start()
        self.command(linear=0.1, seconds=0.1)
        self.node.timer_recalc_odom.cancel()
        def failed_feedback():
            raise RuntimeError('Feedback failed during reset')
        self.driver.left_pos = failed_feedback
        self.assertFalse(self.call('reset_odometry').success)
        self.assertFalse(self.node.controller.connected)
        self.assertEqual((self.driver.left_axis_vel, self.driver.right_axis_vel), (0, 0))

    def test_invalid_calibration_request_stops_instead_of_leaving_motors_active(self):
        self.start()
        self.command(linear=0.1, seconds=0.1)
        self.assertFalse(self.call('calibrate').success)
        self.assertFalse(self.node.controller.connected)
        self.assertEqual((self.driver.left_axis_vel, self.driver.right_axis_vel), (0, 0))


if __name__ == '__main__':
    unittest.main()
