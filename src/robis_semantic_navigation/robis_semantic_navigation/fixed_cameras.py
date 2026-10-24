"""Calibrated fixed-camera estimates and short-lived geometric object tracks.

RGB positions are ray/plane estimates, not measured depths. Plane heights and
map alignment must be supplied explicitly. This module has no ROS or YOLO dependency.
"""
from copy import deepcopy
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
import uuid

import cv2
import numpy as np
import yaml


def finite(value):
    try:
        value = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError('expected a finite number') from error
    if not math.isfinite(value):
        raise ValueError('expected a finite number')
    return value


@dataclass
class Calibration:
    camera_id: str
    width: int
    height: int
    matrix: np.ndarray
    distortion: np.ndarray
    world_from_camera: np.ndarray

    @classmethod
    def load(cls, path, camera_id, image_space):
        if image_space not in ('raw', 'undistorted'):
            raise ValueError('image_space must be raw or undistorted (full frame, no crop)')
        with np.load(path, allow_pickle=False) as data:
            if str(int(data['cam_id'].item())) != str(camera_id):
                raise ValueError('calibration cam_id does not match configured camera')
            matrix = np.array(data['K' if image_space == 'raw' else 'nK'], float)
            distortion = np.array(data['dist'], float).reshape(-1)
            transform = np.array(data['rt_inv'], float)
            inverse = np.array(data['rt'], float)
            width, height = int(data['width'].item()), int(data['height'].item())
        if (matrix.shape != (3, 3) or transform.shape != (4, 4)
                or inverse.shape != (4, 4) or distortion.size not in (4, 5, 8, 12, 14)
                or width <= 0 or height <= 0):
            raise ValueError('invalid calibration dimensions')
        if not all(np.isfinite(a).all() for a in (matrix, distortion, transform, inverse)):
            raise ValueError('non-finite calibration')
        rotation = transform[:3, :3]
        if (matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                or not np.allclose(matrix[2], [0, 0, 1])
                or not np.allclose(transform[3], [0, 0, 0, 1])
                or not np.allclose(transform @ inverse, np.eye(4), atol=1e-6)
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
            raise ValueError('invalid camera intrinsics or rigid extrinsics')
        return cls(str(camera_id), width, height, matrix,
                   distortion if image_space == 'raw' else np.zeros(5), transform)

    def intersect(self, pixel, plane_z, maximum_range):
        u, v = map(finite, pixel)
        if not (0 <= u < self.width and 0 <= v < self.height):
            raise ValueError('anchor outside the calibrated full image')
        ray_xy = cv2.undistortPointsIter(
            np.array([[[u, v]]], np.float64), self.matrix, self.distortion,
            None, None, (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 40, 1e-12),
        ).reshape(2)
        ray = self.world_from_camera[:3, :3] @ np.array([*ray_xy, 1.])
        origin = self.world_from_camera[:3, 3]
        if abs(ray[2]) < 1e-7:
            raise ValueError('ray parallel to the assumed plane')
        distance = (finite(plane_z) - origin[2]) / ray[2]
        if distance <= 0 or distance * np.linalg.norm(ray) > maximum_range:
            raise ValueError('plane behind camera or beyond maximum range')
        point = origin + distance * ray
        if not np.isfinite(point).all():
            raise ValueError('invalid projected point')
        return point


@dataclass
class Track:
    track_id: str
    label: str
    position: list
    confidence: float
    last_seen: float
    observations: int = 0
    views: dict = field(default_factory=dict)
    sensitivity_m: float = 0.
    confirmed: bool = False


class FixedCameraMap:
    def __init__(self, configuration):
        self.path = Path(configuration)
        self.config = yaml.safe_load(self.path.read_text())
        self.environment = str(self.config['environment_id']).strip()
        self.checksum = str(self.config['map_checksum']).strip()
        alignment = self.config['map_from_world']
        if not self.environment or len(self.checksum) != 64:
            raise ValueError('environment_id and navigation-map SHA-256 are required')
        try:
            int(self.checksum, 16)
            x, y, yaw = [finite(alignment[key]) for key in ('x', 'y', 'yaw')]
        except (ValueError, TypeError, KeyError) as error:
            raise ValueError('supply an explicit map_from_world x/y/yaw alignment') from error
        self.rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                                  [math.sin(yaw), math.cos(yaw)]])
        self.translation = np.array([x, y])
        self.ttl = finite(self.config.get('ttl_sec', 3.))
        self.radius = finite(self.config.get('association_radius_m', 0.6))
        self.window = finite(self.config.get('fusion_window_sec', 0.2))
        self.maximum_range = finite(self.config.get('maximum_range_m', 12.))
        self.maximum_sensitivity = finite(self.config.get('maximum_sensitivity_m', 0.6))
        self.minimum_observations = int(self.config.get('minimum_observations', 2))
        self.minimum_cameras = int(self.config.get('minimum_cameras', 1))
        self.pixel_uncertainty = finite(self.config.get('anchor_uncertainty_px', 4.))
        if (min(self.ttl, self.radius, self.maximum_range, self.maximum_sensitivity) <= 0
                or not 0 <= self.window < self.ttl or self.minimum_observations < 1
                or self.pixel_uncertainty < 0 or self.minimum_cameras < 1):
            raise ValueError('invalid tracking or projection limits')
        self.planes = self.config['planes']
        if not isinstance(self.planes, dict) or not self.planes:
            raise ValueError('supply class-specific reference-plane heights')
        for plane in self.planes.values():
            finite(plane['z_m'])
            if finite(plane['height_uncertainty_m']) < 0:
                raise ValueError('height uncertainty must be nonnegative')
            if plane.get('anchor_method', 'bbox_center') not in ('bbox_center', 'mask_incenter'):
                raise ValueError('unsupported image anchor method')
        self.cameras = {}
        hashes = {}
        for item in self.config['cameras']:
            path = self.path.parent / item['calibration']
            calibration = Calibration.load(path, item['id'], item['image_space'])
            if calibration.camera_id in self.cameras:
                raise ValueError('duplicate camera id')
            self.cameras[calibration.camera_id] = calibration
            hashes[calibration.camera_id] = hashlib.sha256(path.read_bytes()).hexdigest()
        if not self.cameras:
            raise ValueError('at least one camera is required')
        self.fingerprint = hashlib.sha256(json.dumps(
            {'config': self.config, 'calibrations': hashes}, sort_keys=True).encode()).hexdigest()
        self.tracks = {}
        self.last_frames = {}
        self.latest_stamp = None
        self.rejections = {}

    def project(self, camera, detection):
        plane = self.planes[detection['label']]
        if plane.get('anchor_method') == 'mask_incenter' and 'anchor' not in detection:
            raise ValueError('segmentation anchor required for this class')
        score = finite(detection['score'])
        if not 0 <= score <= 1:
            raise ValueError('confidence outside [0,1]')
        x1, y1, x2, y2 = map(finite, detection['bbox'])
        if not (0 <= x1 < x2 <= camera.width and 0 <= y1 < y2 <= camera.height):
            raise ValueError('invalid image bounding box')
        anchor = detection.get('anchor', [(x1 + x2) / 2, (y1 + y2) / 2])
        u, v = map(finite, anchor)
        if not (x1 <= u <= x2 and y1 <= v <= y2):
            raise ValueError('anchor outside detection')
        z, dz = finite(plane['z_m']), finite(plane['height_uncertainty_m'])
        point = camera.intersect([u, v], z, self.maximum_range)
        variations = []
        for du, dv, dh in [(self.pixel_uncertainty, 0, 0),
                           (0, self.pixel_uncertainty, 0), (0, 0, dz)]:
            deviations = [np.linalg.norm(camera.intersect(
                [u + sign * du, v + sign * dv], z + sign * dh,
                self.maximum_range)[:2] - point[:2]) for sign in (-1, 1)]
            variations.append(max(deviations))
        sensitivity = float(np.linalg.norm(variations))
        if sensitivity > self.maximum_sensitivity:
            raise ValueError('projection too sensitive to pixel/height uncertainty')
        point[:2] = self.rotation @ point[:2] + self.translation
        return point.tolist(), score, sensitivity

    def ingest(self, event, now=None):
        camera_id = str(event['camera_id'])
        camera = self.cameras[camera_id]
        stamp = finite(event['stamp'])
        now = stamp if now is None else finite(now)
        index = int(event['frame_index'])
        if index < 0 or index != event['frame_index']:
            raise ValueError('frame_index must be a nonnegative integer')
        if (event['width'], event['height']) != (camera.width, camera.height):
            raise ValueError('resolution differs from calibration; resize/crop is unsupported')
        if stamp > now + 0.05 or now - stamp > self.ttl:
            raise ValueError('future or expired camera frame')
        previous = self.last_frames.get(camera_id)
        if (previous and (index <= previous[0] or stamp <= previous[1])):
            raise ValueError('duplicate or out-of-order frame')
        if self.latest_stamp is not None and stamp < self.latest_stamp:
            raise ValueError('camera frames must be merged in timestamp order')
        self.expire(now)
        projected = []
        for detection in event['detections']:
            try:
                point, score, sensitivity = self.project(camera, detection)
                projected.append((detection['label'], point, score, sensitivity))
            except (KeyError, ValueError, cv2.error) as error:
                reason = str(error)
                self.rejections[reason] = self.rejections.get(reason, 0) + 1
        # Each track can consume at most one detection from this camera frame.
        # A tentative duplicate from one view must not steal the next view
        # from an established identity merely by being a few pixels closer.
        # The distance gate and one detection per track/frame still apply.
        pairs = sorted((not t.confirmed, math.dist(t.position[:2], p[:2]), i, t.track_id)
                       for i, (label, p, _, _) in enumerate(projected)
                       for t in self.tracks.values() if t.label == label
                       and math.dist(t.position[:2], p[:2]) <= self.radius)
        assignments, used = {}, set()
        for _, _, i, track_id in pairs:
            if i not in assignments and track_id not in used:
                assignments[i] = track_id
                used.add(track_id)
        for i, (label, position, confidence, sensitivity) in enumerate(projected):
            if i in assignments:
                track = self.tracks[assignments[i]]
            else:
                track = Track(str(uuid.uuid4()), label, position, confidence, stamp)
                self.tracks[track.track_id] = track
            track.views = {k: v for k, v in track.views.items()
                           if stamp - v['stamp'] <= self.window}
            track.views[camera_id] = {
                'position': position, 'score': confidence,
                'stamp': stamp, 'sensitivity': sensitivity,
            }
            views = list(track.views.values())
            weights = [max(v['score'], 0.01) for v in views]
            track.position = np.average(
                [v['position'] for v in views], axis=0, weights=weights).tolist()
            track.confidence = float(np.average([v['score'] for v in views], weights=weights))
            track.sensitivity_m = max(v['sensitivity'] for v in views)
            track.last_seen = stamp
            track.observations += 1
            if (track.observations >= self.minimum_observations
                    and len(track.views) >= self.minimum_cameras):
                track.confirmed = True
        self.last_frames[camera_id] = [index, stamp]
        self.latest_stamp = stamp
        return self.snapshot(now)

    def expire(self, now):
        self.tracks = {key: value for key, value in self.tracks.items()
                       if -0.05 <= now - value.last_seen < self.ttl}

    def snapshot(self, now):
        self.expire(finite(now))
        return {'environment_id': self.environment, 'map_checksum': self.checksum,
                'frame_id': 'map', 'stamp': now, 'ttl_sec': self.ttl,
                'objects': [{'id': t.track_id, 'label': t.label, 'position': list(t.position),
                             'confidence': t.confidence, 'last_seen': t.last_seen,
                             'observations': t.observations,
                             'cameras': sorted(t.views), 'sensitivity_m': t.sensitivity_m}
                            for t in self.tracks.values()
                            if t.observations >= self.minimum_observations
                            and len(t.views) >= self.minimum_cameras]}

    def checkpoint(self):
        return deepcopy({'fingerprint': self.fingerprint, 'latest_stamp': self.latest_stamp,
                         'last_frames': self.last_frames,
                         'tracks': [vars(track) for track in self.tracks.values()]})

    def restore(self, document):
        if document['fingerprint'] != self.fingerprint:
            raise ValueError('checkpoint calibration/configuration fingerprint mismatch')
        tracks = [Track(**item) for item in document['tracks']]
        self.tracks = {track.track_id: track for track in tracks}
        self.latest_stamp = document['latest_stamp']
        self.last_frames = document['last_frames']
