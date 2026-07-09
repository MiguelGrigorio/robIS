#!/usr/bin/env python3

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__("usb_camera_node")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("video_device", "/dev/video0")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("framerate", 30.0)
        self.declare_parameter("frame_id", "camera_link")

        self.image_topic = self.get_parameter("image_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.framerate = float(self.get_parameter("framerate").value)

        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.bridge = CvBridge()

        video_device = self.get_parameter("video_device").value
        width = int(self.get_parameter("image_width").value)
        height = int(self.get_parameter("image_height").value)

        pipeline = (
            f"v4l2src device={video_device} ! "
            f"image/jpeg,width={width},height={height},framerate={int(self.framerate)}/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "appsink drop=true max-buffers=1"
        )

        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            raise RuntimeError("Nao foi possivel abrir a camera")

        self.timer = self.create_timer(1.0 / self.framerate, self.publish_frame)
        self.get_logger().info(f"Camera iniciada publicando em {self.image_topic}")

    def publish_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("Falha ao capturar frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap.isOpened():
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


if __name__ == "__main__":
    main()
