#!/usr/bin/env python3
"""Forward bounded manual commands and stop when their stream expires."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class FoxgloveTeleop(Node):
    """Watch the manual input without continuously overriding idle navigation."""

    def __init__(self):
        super().__init__('foxglove_teleop')
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('max_linear_m_s', 0.2)
        self.declare_parameter('max_angular_rad_s', 0.6)
        self.timeout = float(self.get_parameter('timeout_sec').value)
        self.max_linear = float(self.get_parameter('max_linear_m_s').value)
        self.max_angular = float(self.get_parameter('max_angular_rad_s').value)
        if any(not math.isfinite(v) or v <= 0 for v in (
            self.timeout, self.max_linear, self.max_angular,
        )):
            raise ValueError('Teleop limits and timeout must be positive and finite')
        self.active = False
        self.armed = False
        self.last_command = 0.0
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel_teleop', self.receive, 1,
        )
        self.timer = self.create_timer(
            0.05, self.check_timeout, clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def receive(self, message):
        """Accept only planar, finite velocities within the manual limits."""
        values = [getattr(vector, axis) for vector in (message.linear, message.angular)
                  for axis in ('x', 'y', 'z')]
        if not all(math.isfinite(value) for value in values):
            self.stop()
            return
        command = Twist()
        command.linear.x = max(-self.max_linear, min(self.max_linear, message.linear.x))
        command.angular.z = max(-self.max_angular, min(self.max_angular, message.angular.z))
        moving = bool(command.linear.x or command.angular.z)
        if moving and not self.armed:
            self.get_logger().warning(
                'Press Stop in Teleop before moving: ignoring '
                f'linear.x={command.linear.x}, angular.z={command.angular.z}',
                throttle_duration_sec=5.0,
            )
            return
        if not moving:
            self.armed = True
        self.publisher.publish(command)
        self.last_command = time.monotonic()
        self.active = moving

    def check_timeout(self):
        """Publish one stop after a missed heartbeat, using wall-clock time."""
        if self.active and time.monotonic() - self.last_command >= self.timeout:
            self.stop()

    def stop(self):
        """Clear manual motion without latching a permanent zero publisher."""
        self.publisher.publish(Twist())
        self.active = False
        self.armed = False


def main():
    """Run the optional manual command forwarder."""
    rclpy.init()
    node = None
    try:
        node = FoxgloveTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
