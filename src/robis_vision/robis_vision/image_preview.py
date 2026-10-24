"""Publish a bounded JPEG preview without changing the perception image stream."""

import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class ImagePreview(Node):
    """Keep only the newest raw image and encode at a limited wall-clock rate."""

    def __init__(self):
        super().__init__('foxglove_image_preview')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_raw/compressed')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('opencv_threads', 1)
        self.declare_parameter('input_reliability', 'reliable')
        reliability = self.get_parameter('input_reliability').value
        policies = {'reliable': ReliabilityPolicy.RELIABLE,
                    'best_effort': ReliabilityPolicy.BEST_EFFORT}
        if reliability not in policies:
            raise ValueError('input_reliability must be reliable or best_effort')
        threads = int(self.get_parameter('opencv_threads').value)
        if threads < 1:
            raise ValueError('opencv_threads must be positive')
        cv2.setNumThreads(threads)
        fps = float(self.get_parameter('fps').value)
        self.quality = int(self.get_parameter('jpeg_quality').value)
        if not math.isfinite(fps) or fps <= 0 or not 1 <= self.quality <= 100:
            raise ValueError('fps must be positive and finite; jpeg_quality must be 1..100')
        self.latest = None
        self.bridge = CvBridge()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(
            CompressedImage, self.get_parameter('output_topic').value, qos,
        )
        self.subscription = self.create_subscription(
            Image, self.get_parameter('input_topic').value, self.receive,
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=policies[reliability]),
        )
        self.timer = self.create_timer(
            1.0 / fps, self.publish_preview, clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def receive(self, image):
        """Replace the pending image instead of accumulating a frame queue."""
        self.latest = image

    def publish_preview(self):
        """Encode only when a viewer subscribes, preserving the source header."""
        image, self.latest = self.latest, None
        if image is None or self.publisher.get_subscription_count() == 0:
            return
        try:
            pixels = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            success, encoded = cv2.imencode(
                '.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, self.quality],
            )
            if not success:
                raise ValueError('JPEG encoder returned no image')
        except (CvBridgeError, cv2.error, ValueError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        preview = CompressedImage()
        preview.header = image.header
        preview.format = 'bgr8; jpeg compressed bgr8'
        preview.data = encoded.tobytes()
        self.publisher.publish(preview)


def main(args=None):
    """Run the observation-only image publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = ImagePreview()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
