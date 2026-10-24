"""Replay recorded fixed-camera images or explicitly selected recorded detections."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time

import cv2
import numpy as np

from .fixed_cameras import FixedCameraMap
from .image_anchors import yolo_detections


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'w') as stream:
            json.dump(value, stream, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def ros_message(snapshot, source_id, epoch_ns, first_stamp):
    from builtin_interfaces.msg import Time
    from robis_interfaces.msg import ExternalObject, ExternalObjects
    from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose

    def stamp(value):
        nanoseconds = epoch_ns + round((value - first_stamp) * 1e9)
        return Time(sec=nanoseconds // 10**9, nanosec=nanoseconds % 10**9)

    message = ExternalObjects()
    message.header.frame_id = 'map'
    message.header.stamp = stamp(snapshot['stamp'])
    message.environment_id = snapshot['environment_id']
    message.map_checksum = snapshot['map_checksum']
    message.source_id = source_id
    message.validity_sec = float(snapshot['ttl_sec'])
    for item in snapshot['objects']:
        detection = Detection3D()
        detection.id = item['id']
        detection.header.frame_id = 'map'
        detection.header.stamp = stamp(item['last_seen'])
        point = detection.bbox.center.position
        point.x, point.y, point.z = map(float, item['position'])
        detection.bbox.center.orientation.w = 1.
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = item['label']
        result.hypothesis.score = item['confidence']
        result.pose.pose = detection.bbox.center
        # Sensitivity is an estimate, not a statistically calibrated covariance.
        detection.results.append(result)
        observation = ExternalObject()
        observation.detection = detection
        observation.revision = item['observations']
        observation.projection_sensitivity_m = item['sensitivity_m']
        message.objects.append(observation)
    return message


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--checkpoint', type=Path)
    parser.add_argument('--detections-only', action='store_true',
                        help='use recorded detections; no image inference is performed')
    parser.add_argument('--model', type=Path, default=Path('/opt/robis-models/yolo26n-seg.pt'))
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--image-size', type=int, default=640)
    parser.add_argument('--publish-ros', action='store_true', help='publish at recorded speed, no motion')
    parser.add_argument('--use-sim-time', action='store_true', help='pace publication against ROS /clock')
    parser.add_argument('--source-id', default='labsea-fixed-cameras')
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output already exists; select a new output file')
    engine = FixedCameraMap(args.config)
    model_hash = None
    if not args.detections_only:
        if not args.model.is_file():
            parser.error('model weights must exist locally; downloads at runtime are disabled')
        model_hash = hashlib.sha256(args.model.read_bytes()).hexdigest()
    if args.checkpoint and args.checkpoint.exists():
        checkpoint = json.loads(args.checkpoint.read_text())
        if checkpoint.get('detector_model_sha256') != model_hash:
            parser.error('checkpoint detector model differs from the selected model')
        engine.restore(checkpoint)
    model = None
    if not args.detections_only:
        if not args.model.is_file():
            parser.error('model weights must exist locally; downloads at runtime are disabled')
        Path(os.environ.get('YOLO_CONFIG_DIR', '/tmp/robis-yolo')).mkdir(parents=True, exist_ok=True)
        from ultralytics import YOLO
        import torch
        torch.set_num_threads(2)
        model = YOLO(str(args.model))
        model.predict(np.zeros((640, 640, 3), np.uint8), imgsz=args.image_size,
                      device=args.device, verbose=False)
    node, publisher = None, None
    if args.publish_ros:
        import rclpy
        from robis_interfaces.msg import ExternalObjects
        from rclpy.parameter import Parameter
        rclpy.init(args=[])
        node = rclpy.create_node('fixed_camera_replay', parameter_overrides=[
            Parameter('use_sim_time', value=args.use_sim_time)])
        publisher = node.create_publisher(ExternalObjects, '/semantic_navigation/external_objects', 10)
        # Give the reader discovery time before the first finite recording frame.
        discovery_until = time.monotonic() + 1.
        while time.monotonic() < discovery_until:
            rclpy.spin_once(node, timeout_sec=0.05)
        startup_deadline = time.monotonic() + 15.
        while node.get_clock().now().nanoseconds == 0:
            if time.monotonic() > startup_deadline:
                raise ValueError('ROS /clock is unavailable')
            rclpy.spin_once(node, timeout_sec=0.05)
    metadata = {'record_type': 'provenance', 'configuration_sha256': engine.fingerprint,
                'input_sha256': hashlib.sha256(args.input.read_bytes()).hexdigest(),
                'detector': 'recorded_detections' if model is None else 'ultralytics_yolo',
                'model_sha256': model_hash,
                'projection': 'class_reference_plane_estimate', 'physical_validation': False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    first_stamp, epoch_ns = None, None
    frame_count, peak_objects = 0, 0
    try:
        with args.input.open() as inputs, args.output.open('x') as output:
            output.write(json.dumps(metadata) + '\n')
            for line in inputs:
                if not line.strip():
                    continue
                event = json.loads(line)
                if first_stamp is None:
                    first_stamp = float(event['stamp'])
                    epoch_ns = node.get_clock().now().nanoseconds if node else 0
                if publisher:
                    deadline = epoch_ns + round((float(event['stamp']) - first_stamp) * 1e9)
                    while node.get_clock().now().nanoseconds < deadline:
                        rclpy.spin_once(node, timeout_sec=0.02)
                latency = None
                image_hash = None
                if model is not None:
                    path = args.input.parent / event['image']
                    image = cv2.imread(str(path))
                    if image is None:
                        raise ValueError('cannot decode image: ' + str(path))
                    if (image.shape[1], image.shape[0]) != (event['width'], event['height']):
                        raise ValueError('recorded dimensions do not match the image')
                    image_hash = hashlib.sha256(path.read_bytes()).hexdigest()
                    if str(args.device) != 'cpu' and torch.cuda.is_available():
                        torch.cuda.synchronize()
                    started = time.perf_counter()
                    result = model.predict(image, imgsz=args.image_size, conf=0.25,
                                           device=args.device, retina_masks=True, verbose=False)[0]
                    if str(args.device) != 'cpu' and torch.cuda.is_available():
                        torch.cuda.synchronize()
                    latency = (time.perf_counter() - started) * 1000
                    event['detections'] = yolo_detections(result, engine.planes, event['width'], event['height'])
                snapshot = engine.ingest(event)
                frame_count += 1
                peak_objects = max(peak_objects, len(snapshot['objects']))
                snapshot.update(inference_ms=latency, image_sha256=image_hash,
                                camera_id=str(event['camera_id']), frame_index=event['frame_index'],
                                detections=event['detections'], image=event.get('image'))
                output.write(json.dumps(snapshot, allow_nan=False) + '\n')
                output.flush()
                if publisher:
                    publisher.publish(ros_message(snapshot, args.source_id, epoch_ns, first_stamp))
                    rclpy.spin_once(node, timeout_sec=0.)
                if args.checkpoint:
                    checkpoint = engine.checkpoint()
                    checkpoint['detector_model_sha256'] = model_hash
                    atomic_json(args.checkpoint, checkpoint)
            # Final expiry is explicit in the recording, not inferred from a missing frame.
            if engine.latest_stamp is not None:
                final_stamp = engine.latest_stamp + engine.ttl + 1e-6
                if publisher:
                    deadline = epoch_ns + round((final_stamp - first_stamp) * 1e9)
                    while node.get_clock().now().nanoseconds < deadline:
                        rclpy.spin_once(node, timeout_sec=0.02)
                final = engine.snapshot(final_stamp)
                output.write(json.dumps(final) + '\n')
                if publisher:
                    publisher.publish(ros_message(final, args.source_id, epoch_ns, first_stamp))
            output.write(json.dumps({'record_type': 'rejections', 'counts': engine.rejections}) + '\n')
            if not frame_count:
                raise ValueError('the recording contains no camera frames')
    finally:
        if node:
            node.destroy_node()
            rclpy.shutdown()
    print(json.dumps({'frames_processed': frame_count, 'maximum_confirmed_objects': peak_objects,
                      'detector': metadata['detector'], 'output': str(args.output)}), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
