#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class BallFollower(Node):
    def __init__(self):
        super().__init__("ball_follower")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("debug_image_topic", "/ball_follower/debug_image")
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("target_h_min", 5)
        self.declare_parameter("target_h_max", 25)
        self.declare_parameter("target_s_min", 80)
        self.declare_parameter("target_v_min", 80)
        self.declare_parameter("min_area_px", 350.0)

        self.declare_parameter("target_radius_px", 70.0)
        self.declare_parameter("radius_deadband_px", 10.0)
        self.declare_parameter("center_deadband_px", 35.0)

        self.declare_parameter("max_linear_speed", 0.5)  # previously 0.22
        self.declare_parameter("max_angular_speed", 1.5)  # previously 0.9
        self.declare_parameter("linear_gain", 0.01)  # previously 0.004
        self.declare_parameter("angular_gain", 1.4)
        self.declare_parameter("search_angular_speed", 0.25)
        self.declare_parameter("stop_on_lost", True)

        self.bridge = CvBridge()
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)

        image_topic = self.get_parameter("image_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        debug_image_topic = self.get_parameter("debug_image_topic").value

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.debug_pub = (
            self.create_publisher(Image, debug_image_topic, 10)
            if self.publish_debug_image
            else None
        )
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10,
        )

        self.get_logger().info(
            f"Seguidor de bola lendo {image_topic} e comandando {cmd_vel_topic}"
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        height, width = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(
            [
                int(self.get_parameter("target_h_min").value),
                int(self.get_parameter("target_s_min").value),
                int(self.get_parameter("target_v_min").value),
            ],
            dtype=np.uint8,
        )
        upper = np.array(
            [
                int(self.get_parameter("target_h_max").value),
                255,
                255,
            ],
            dtype=np.uint8,
        )

        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist()

        if not contours:
            self.publish_lost_target(twist)
            self.publish_debug(msg, frame)
            return

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        min_area = float(self.get_parameter("min_area_px").value)

        if area < min_area:
            self.publish_lost_target(twist)
            self.publish_debug(msg, frame)
            return

        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
        center_error_px = center_x - (width / 2.0)
        radius_error_px = float(self.get_parameter("target_radius_px").value) - radius

        center_deadband = float(self.get_parameter("center_deadband_px").value)
        radius_deadband = float(self.get_parameter("radius_deadband_px").value)

        if abs(center_error_px) > center_deadband:
            angular_gain = float(self.get_parameter("angular_gain").value)
            max_angular = float(self.get_parameter("max_angular_speed").value)
            twist.angular.z = clamp(
                -angular_gain * center_error_px / (width / 2.0),
                -max_angular,
                max_angular,
            )

        if abs(radius_error_px) > radius_deadband:
            linear_gain = float(self.get_parameter("linear_gain").value)
            max_linear = float(self.get_parameter("max_linear_speed").value)
            twist.linear.x = clamp(
                linear_gain * radius_error_px,
                -max_linear,
                max_linear,
            )

        self.cmd_pub.publish(twist)

        if self.publish_debug_image:
            cv2.circle(frame, (int(center_x), int(center_y)), int(radius), (0, 255, 0), 2)
            cv2.line(
                frame,
                (width // 2, 0),
                (width // 2, height),
                (255, 0, 0),
                1,
            )
            self.publish_debug(msg, frame)

    def publish_lost_target(self, twist):
        if bool(self.get_parameter("stop_on_lost").value):
            twist.angular.z = 0.0
        else:
            twist.angular.z = float(self.get_parameter("search_angular_speed").value)

        self.cmd_pub.publish(twist)

    def publish_debug(self, source_msg, frame):
        if not self.publish_debug_image:
            return

        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug_msg.header = source_msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
