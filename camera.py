#!/usr/bin/env python3

import cv2

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class UsbCameraNode(Node):

    def __init__(self):
        super().__init__('usb_camera_node')

        self.publisher = self.create_publisher(
            Image,
            '/image_raw',
            10
        )

        self.bridge = CvBridge()

        pipeline = (
            "v4l2src device=/dev/video0 ! "
            "image/jpeg,width=640,height=480,framerate=30/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "appsink drop=true max-buffers=1"
        )

        self.cap = cv2.VideoCapture(
            pipeline,
            cv2.CAP_GSTREAMER
        )

        if not self.cap.isOpened():
            raise RuntimeError("Não foi possível abrir a câmera")

        self.timer = self.create_timer(
            1.0 / 30.0,
            self.publish_frame
        )

        self.get_logger().info("Camera iniciada")

    def publish_frame(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("Falha ao capturar frame")
            return

        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)

    def destroy_node(self):

        if self.cap.isOpened():
            self.cap.release()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = UsbCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()