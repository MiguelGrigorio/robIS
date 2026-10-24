#!/usr/bin/env python3
"""USB camera publisher with synchronized Image and CameraInfo messages."""

import math

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .camera_calibration import load_calibration


class UsbCameraNode(Node):
    """Capture USB frames using GStreamer or V4L2."""

    def __init__(self):
        super().__init__("usb_camera_node")
        defaults = {
            "image_topic": "/camera/image_raw",
            "camera_info_topic": "/camera/camera_info",
            "video_device": "/dev/video0",
            "image_width": 640, "image_height": 480, "framerate": 30.0,
            "frame_id": "camera_optical_frame",
            "capture_backend": "v4l2", "camera_info_url": "", "opencv_threads": 1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        self.frame_id = values["frame_id"]
        self.width = int(values["image_width"])
        self.height = int(values["image_height"])
        self.framerate = float(values["framerate"])
        if int(values["opencv_threads"]) < 1:
            raise ValueError('opencv_threads must be positive')
        cv2.setNumThreads(int(values["opencv_threads"]))
        if (self.width <= 0 or self.height <= 0
                or not math.isfinite(self.framerate) or self.framerate <= 0):
            raise ValueError("Image dimensions and framerate must be positive")
        if int(self.framerate) != self.framerate:
            raise ValueError("Use a whole-number framerate for the USB capture pipeline")
        self.calibration = load_calibration(
            values["camera_info_url"], self.width, self.height)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, values["image_topic"], 10)
        self.info_publisher = self.create_publisher(
            CameraInfo, values["camera_info_topic"], 10)
        self.cap = self._open_capture(
            str(values["video_device"]), values["capture_backend"])
        self.timer = self.create_timer(1.0 / self.framerate, self.publish_frame)
        calibration_status = "calibrada" if self.calibration else "sem calibração (K[0]=0)"
        self.get_logger().info(
            f"Câmera em {values['image_topic']}; {calibration_status}")

    def _open_capture(self, device, backend):
        """Preserve the existing MJPEG pipeline and fall back to V4L2."""
        if backend not in ("auto", "gstreamer", "v4l2"):
            raise ValueError("capture_backend must be auto, gstreamer or v4l2")
        if backend in ("auto", "gstreamer"):
            # Pipeline strings are not a shell, but device names must remain paths.
            if not device.startswith("/dev/") or any(char.isspace() for char in device):
                raise ValueError("video_device must be a /dev/ path without spaces")
            pipeline = (
                f"v4l2src device={device} ! "
                f"image/jpeg,width={self.width},height={self.height},"
                f"framerate={int(self.framerate)}/1 ! jpegdec ! "
                "videoconvert ! appsink drop=true max-buffers=1"
            )
            capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if capture.isOpened():
                return capture
            capture.release()
            if backend == "gstreamer":
                raise RuntimeError("Não foi possível abrir a câmera por GStreamer")
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.framerate)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError("Não foi possível abrir a câmera USB")
        return capture

    def publish_frame(self):
        """Publish the same optical frame and timestamp for both messages."""
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning(
                "Falha ao capturar frame", throttle_duration_sec=5.0)
            return
        height, width = frame.shape[:2]
        if self.calibration and (width != self.width or height != self.height):
            self.get_logger().error(
                "Resolução capturada difere da calibração; frame descartado",
                throttle_duration_sec=5.0,
            )
            return
        image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = self.frame_id
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = width, height
        if self.calibration:
            for key, value in self.calibration.items():
                setattr(info, key, value)
        # ROS identifies an uncalibrated camera by K[0] == 0, left at its default.
        self.info_publisher.publish(info)
        self.publisher.publish(image)

    def destroy_node(self):
        """Release the camera before disposing of ROS resources."""
        if hasattr(self, "cap"):
            self.cap.release()
        return super().destroy_node()


def main(args=None):
    """Run the camera and clean up on Ctrl+C or capture errors."""
    rclpy.init(args=args)
    node = None
    try:
        node = UsbCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
