#!/usr/bin/env python3
"""Check Gazebo sensors/TF and optionally test a short simulated movement."""

import argparse
import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class SimulationCheck(Node):
    """Observe the simulated stack; never start any hardware driver."""

    def __init__(self):
        super().__init__("robis_simulation_check")
        self.messages = {}
        self.counts = {}
        self.received_at = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        for topic, message_type in {
            "/clock": Clock,
            "/odom": Odometry,
            "/scan": LaserScan,
            "/camera/image_raw": Image,
            "/camera/camera_info": CameraInfo,
            "/joint_states": JointState,
        }.items():
            self.counts[topic] = 0
            self.create_subscription(
                message_type, topic,
                lambda message, name=topic: self.receive(name, message),
                # Match the reliable raw-image publishers and perception nodes.
                # Large best-effort samples can stall at cold start on CycloneDDS.
                1 if message_type is Image else qos_profile_sensor_data,
            )

    def receive(self, topic, message):
        self.messages[topic] = message
        self.counts[topic] += 1
        self.received_at[topic] = time.monotonic()

    def ready(self):
        if any(self.counts[topic] < 2 for topic in self.counts):
            return False
        if any(time.monotonic() - value > 5 for value in self.received_at.values()):
            return False
        joints = self.messages["/joint_states"]
        if not {
            "left_wheel_joint", "right_wheel_joint",
            "front_caster_joint", "front_caster_wheel_joint",
            "rear_caster_joint", "rear_caster_wheel_joint",
        }.issubset(joints.name):
            return False
        info = self.messages["/camera/camera_info"]
        image = self.messages["/camera/image_raw"]
        scan = self.messages["/scan"]
        if not (
            info.k[0] > 0 and image.width > 0 and len(scan.ranges) > 0
            and any(math.isfinite(value) for value in scan.ranges)
            and (image.width, image.height) == (info.width, info.height)
            and image.header.frame_id == info.header.frame_id == "camera_optical_frame"
            and scan.header.frame_id == "lidar_link"
        ):
            return False
        for target, source in (
            ("odom", "base_footprint"),
            ("base_footprint", "lidar_link"),
            ("base_footprint", "camera_optical_frame"),
            ("odom", image.header.frame_id),
            ("odom", scan.header.frame_id),
            ("base_footprint", "front_caster_wheel"),
            ("base_footprint", "rear_caster_wheel"),
        ):
            try:
                self.buffer.lookup_transform(target, source, Time())
            except TransformException:
                return False
        return True

    def wait_ready(self, timeout):
        deadline = time.monotonic() + timeout
        ready_since = None
        initial_images = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.ready():
                if ready_since is None:
                    ready_since = time.monotonic()
                    initial_images = self.counts['/camera/image_raw']
                if (time.monotonic() - ready_since >= 1.0
                        and self.counts['/camera/image_raw'] >= initial_images + 2):
                    return
            else:
                ready_since = None
        missing = [topic for topic, count in self.counts.items() if count < 2]
        raise RuntimeError("Sensores ou TF indisponíveis: " + str(missing or "verifique frames, TF, resolução e CameraInfo"))

    def simulation_seconds(self):
        stamp = self.messages["/clock"].clock
        return stamp.sec + stamp.nanosec / 1e9

    def move(self):
        # Require a running simulation and its explicit ROS->Gazebo bridge.
        bridges = self.get_subscriptions_info_by_topic("/cmd_vel")
        if not any(item.node_name == "cmd_vel_bridge" for item in bridges):
            raise RuntimeError("Bridge do Gazebo ausente; teste de movimento recusado")
        initial = self.messages["/odom"].pose.pose.position
        x, y = initial.x, initial.y
        command = Twist()
        command.linear.x = 0.08
        try:
            self.publish_for(command, 1.5)
            position = self.messages["/odom"].pose.pose.position
            distance = math.hypot(position.x - x, position.y - y)
            if distance < 0.02:
                raise RuntimeError(f"Movimento não confirmado na odometria: {distance:.4f} m")
            yaw = self.yaw()
            command = Twist()
            command.angular.z = 0.2
            self.publish_for(command, 1.0)
            angle = math.atan2(math.sin(self.yaw() - yaw), math.cos(self.yaw() - yaw))
            if angle < 0.05:
                raise RuntimeError(f"Giro positivo não confirmado: {angle:.4f} rad")
            self.publish_for(Twist(), 0.3)
            velocity = self.messages["/odom"].twist.twist
            if abs(velocity.linear.x) > 0.01 or abs(velocity.angular.z) > 0.02:
                raise RuntimeError("Parada não confirmada na odometria")
        finally:
            for _ in range(5):
                self.publisher.publish(Twist())
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.05)
        return {"distance_m": distance, "rotation_rad": angle, "stopped": True}

    def yaw(self):
        q = self.messages["/odom"].pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def publish_for(self, command, seconds):
        started = self.simulation_seconds()
        deadline = time.monotonic() + 30
        while self.simulation_seconds() - started < seconds:
            if time.monotonic() > deadline:
                raise RuntimeError("Relógio de simulação não avançou no prazo")
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def report(self):
        result = {"messages": self.counts}
        if "/scan" in self.messages:
            scan = self.messages["/scan"]
            result["scan"] = {
                "frame": scan.header.frame_id,
                "samples": len(scan.ranges),
                "finite_ranges": sum(math.isfinite(value) for value in scan.ranges),
            }
        if "/camera/image_raw" in self.messages:
            image = self.messages["/camera/image_raw"]
            result["camera"] = {
                "frame": image.header.frame_id,
                "width": image.width, "height": image.height,
                "last_received_age_sec": time.monotonic() - self.received_at['/camera/image_raw'],
            }
        if "/joint_states" in self.messages:
            joints = self.messages["/joint_states"]
            result["joint_positions_rad"] = dict(zip(joints.name, joints.position))
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=90)
    parser.add_argument("--move", action="store_true", help="Movimento curto no Gazebo")
    options = parser.parse_args()
    rclpy.init(args=[])
    node = SimulationCheck()
    result = {}
    try:
        node.wait_ready(options.timeout)
        if options.move:
            result.update(node.move())
        result["success"] = True
    except (RuntimeError, KeyboardInterrupt) as error:
        result.update(success=False, error=str(error))
    finally:
        result.update(node.report())
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
