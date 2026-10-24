"""Process synchronized ROS RGB views without renewing their capture timestamps."""
import hashlib
import json
import os
import math
from pathlib import Path
import time

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from robis_interfaces.msg import ExternalObjects
from .fixed_camera_replay import ros_message
from .fixed_cameras import FixedCameraMap
from .image_anchors import yolo_detections
from .self_filter import overlaps_robot, robot_polygon, transform_matrix


class SynchronizedImages:
    """Bound pending frames and consume only the newest complete capture."""

    def __init__(self, cameras, maximum_batches=4):
        self.cameras = set(cameras)
        self.maximum_batches = maximum_batches
        self.pending = {}
        self.last_processed = -1
        self.dropped = 0

    def add(self, camera_id, stamp_ns, message):
        if camera_id not in self.cameras or stamp_ns <= self.last_processed:
            return
        self.pending.setdefault(stamp_ns, {})[camera_id] = message
        while len(self.pending) > self.maximum_batches:
            del self.pending[min(self.pending)]
            self.dropped += 1

    def take_latest(self):
        batch = self.peek_latest()
        if batch is None:
            return None
        stamp, views = batch
        self.dropped += sum(key < stamp for key in self.pending)
        self.pending = {key: value for key, value in self.pending.items() if key > stamp}
        self.last_processed = stamp
        return stamp, views

    def peek_latest(self):
        complete = [stamp for stamp, views in self.pending.items()
                    if set(views) == self.cameras]
        if not complete:
            return None
        stamp = max(complete)
        return stamp, self.pending[stamp]


class FixedCameraStream(Node):
    def __init__(self):
        super().__init__('fixed_camera_stream')
        for name, default in [('configuration', ''), ('model', ''), ('device', 'cpu'),
                              ('image_size', 1280), ('confidence', 0.25), ('source_id', 'labsea-fixed-live'),
                              ('output_directory', ''), ('exclude_robot', False),
                              ('robot_frame', 'base_footprint'), ('robot_length', 0.6),
                              ('robot_width', 0.6), ('robot_height', 0.32), ('robot_pose_margin', 0.15)]:
            self.declare_parameter(name, default)
        self.engine = FixedCameraMap(self.get_parameter('configuration').value)
        self.exclude_robot = self.get_parameter('exclude_robot').value
        self.dimensions = [self.get_parameter('robot_' + axis).value for axis in ['length', 'width', 'height']]
        if not np.isfinite(self.dimensions).all() or min(self.dimensions) <= 0:
            raise ValueError('robot volume must have finite positive dimensions')
        margin = self.get_parameter('robot_pose_margin').value
        if not math.isfinite(margin) or margin < 0:
            raise ValueError('robot_pose_margin must be finite and nonnegative')
        self.dimensions[:2] = [value + 2 * margin for value in self.dimensions[:2]]
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) if self.exclude_robot else None
        self.self_filtered = self.tf_waits = 0
        model_path = Path(self.get_parameter('model').value)
        if not model_path.is_file():
            raise ValueError('model weights must exist locally; runtime downloads are disabled')
        self.device = self.get_parameter('device').value
        self.image_size = self.get_parameter('image_size').value
        self.confidence = self.get_parameter('confidence').value
        if not math.isfinite(self.confidence) or not 0 < self.confidence <= 1:
            raise ValueError('confidence must be finite and in (0, 1]')
        Path(os.environ.get('YOLO_CONFIG_DIR', '/tmp/robis-yolo')).mkdir(parents=True, exist_ok=True)
        from ultralytics import YOLO
        import torch
        self.torch = torch
        torch.set_num_threads(2)
        self.model = YOLO(str(model_path))
        self.model.predict(np.zeros((720, 1280, 3), np.uint8), imgsz=self.image_size,
                           device=self.device, verbose=False)
        self.bridge = CvBridge()
        self.frames = SynchronizedImages(self.engine.cameras)
        self.validated_cameras = set()
        self.batches = 0
        self.expired_batches = 0
        self.publisher = self.create_publisher(ExternalObjects, '/semantic_navigation/external_objects', 10)
        self.status = self.create_publisher(String, '/semantic_navigation/fixed_camera_status', 10)
        self.recording = None
        self.directory = None
        output = self.get_parameter('output_directory').value
        if output:
            self.directory = Path(output)
            self.directory.mkdir(parents=True, exist_ok=True)
            self.recording = (self.directory / 'observations.jsonl').open('x')
            self.record({'record_type': 'provenance', 'configuration_sha256': self.engine.fingerprint,
                         'model_sha256': hashlib.sha256(model_path.read_bytes()).hexdigest(),
                         'timestamp_policy': 'original ROS capture time; no rebasing',
                         'synchronization': 'exact capture timestamps', 'inference_image_size': self.image_size,
                         'confidence_threshold': self.confidence})
            self.record({'record_type': 'self_filter', 'enabled': self.exclude_robot,
                         'robot_frame': self.get_parameter('robot_frame').value, 'dimensions_m': self.dimensions,
                         'pose_source': 'ROS TF at the RGB capture timestamp'})
        for config in self.engine.config['cameras']:
            camera_id = str(config['id'])
            prefix = '/fixed/camera_' + camera_id
            self.create_subscription(CameraInfo, config.get('camera_info_topic', prefix + '/camera_info'),
                                     lambda msg, key=camera_id: self.receive_info(key, msg), 2)
            self.create_subscription(Image, config.get('image_topic', prefix + '/image_raw'),
                                     lambda msg, key=camera_id: self.receive_image(key, msg), 2)
        self.create_timer(0.05, self.process)

    def record(self, document):
        if self.recording:
            self.recording.write(json.dumps(document, allow_nan=False) + '\n')
            self.recording.flush()

    def receive_info(self, camera_id, message):
        camera = self.engine.cameras[camera_id]
        valid = ((message.width, message.height) == (camera.width, camera.height)
                 and np.allclose(np.asarray(message.k).reshape(3, 3), camera.matrix, atol=1e-3)
                 and len(message.d) == len(camera.distortion)
                 and np.allclose(message.d, camera.distortion, atol=1e-8))
        if valid:
            self.validated_cameras.add(camera_id)
        else:
            self.validated_cameras.discard(camera_id)
            self.get_logger().error('CameraInfo differs from calibration: ' + camera_id,
                                    throttle_duration_sec=5.)

    def receive_image(self, camera_id, message):
        camera = self.engine.cameras[camera_id]
        if ((message.width, message.height) != (camera.width, camera.height)
                or camera_id not in self.validated_cameras):
            return
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp > 0:
            self.frames.add(camera_id, stamp, message)

    def process(self):
        batch = self.frames.peek_latest()
        if batch is None:
            return
        stamp_ns, views = batch
        stamp = stamp_ns / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        if not -0.05 <= now - stamp < self.engine.ttl:
            self.frames.take_latest()
            self.expired_batches += 1
            return
        robot_tf = None
        if self.exclude_robot:
            try:
                robot_tf = transform_matrix(self.tf_buffer.lookup_transform(
                    'map', self.get_parameter('robot_frame').value,
                    rclpy.time.Time.from_msg(next(iter(views.values())).header.stamp)).transform)
            except TransformException:
                self.tf_waits += 1
                return  # Give TF callbacks a chance to catch up; do not block the executor.
        self.frames.take_latest()
        inference = []
        snapshot = None
        for camera_id, message in sorted(views.items()):
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            if self.device != 'cpu' and self.torch.cuda.is_available():
                self.torch.cuda.synchronize()
            started = time.perf_counter()
            result = self.model.predict(image, imgsz=self.image_size, device=self.device,
                                        conf=self.confidence, retina_masks=True, verbose=False)[0]
            if self.device != 'cpu' and self.torch.cuda.is_available():
                self.torch.cuda.synchronize()
            latency = (time.perf_counter() - started) * 1000
            inference.append(latency)
            event = {'camera_id': camera_id, 'stamp': stamp, 'frame_index': stamp_ns,
                     'width': message.width, 'height': message.height,
                     'detections': yolo_detections(result, self.engine.planes, message.width, message.height)}
            rejected = []
            polygon = None
            if robot_tf is not None:
                polygon = robot_polygon(self.engine.cameras[camera_id], robot_tf,
                                        self.engine.rotation, self.engine.translation, self.dimensions)
                rejected = [d for d in event['detections'] if overlaps_robot(d, polygon)]
                event['detections'] = [d for d in event['detections'] if not overlaps_robot(d, polygon)]
                self.self_filtered += len(rejected)
            snapshot = self.engine.ingest(event, now=max(now, stamp))
            self.record({'record_type': 'frame', **event, 'inference_ms': latency,
                         'self_filtered_detections': rejected,
                         'robot_polygon': polygon.reshape(-1, 2).tolist() if polygon is not None else None,
                         'map_from_robot': robot_tf.tolist() if robot_tf is not None else None,
                         'rgb_sha256': hashlib.sha256(image.tobytes()).hexdigest()})
            if self.directory and self.batches < 2:
                cv2.imwrite(str(self.directory / f'camera_{camera_id}_{self.batches}.png'), image)
                cv2.imwrite(str(self.directory / f'camera_{camera_id}_{self.batches}_debug.png'), result.plot())
        self.publisher.publish(ros_message(snapshot, self.get_parameter('source_id').value, 0, 0.))
        self.batches += 1
        status = {'record_type': 'snapshot', **snapshot, 'batches': self.batches,
                  'inference_ms': inference, 'capture_age_before_inference_sec': now - stamp,
                  'dropped_batches': self.frames.dropped, 'expired_batches': self.expired_batches,
                  'self_filtered_detections': self.self_filtered, 'tf_waits': self.tf_waits,
                  'projection_rejections': dict(self.engine.rejections)}
        self.record(status)
        self.status.publish(String(data=json.dumps(status)))
        if self.batches == 1 or self.batches % 10 == 0:
            self.get_logger().info(f'{self.batches} synchronized batches; {len(snapshot["objects"])} objects')

    def destroy_node(self):
        if self.recording:
            self.recording.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FixedCameraStream()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
