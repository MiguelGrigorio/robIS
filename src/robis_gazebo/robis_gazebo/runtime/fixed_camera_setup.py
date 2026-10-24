#!/usr/bin/env python3
"""Record Gazebo ceiling RGB with supplied extrinsics and full-frame nK.

This is a synthetic fixture: no robot, physical sensors, depth or label images.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from transforms3d.euler import mat2euler
import yaml


def add(parent, tag, value):
    element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def write_fixture_configuration(output, calibrations, navigation_map=None):
    """Explicit test-world assumptions; never a default for physical cameras."""
    (output/'calib').mkdir(exist_ok=True)
    for path in calibrations.glob('calib_rt*.npz'):
        shutil.copy2(path, output/'calib'/path.name)
    map_path = navigation_map
    if map_path is None:
        grid = np.full((160, 160), 254, np.uint8)
        grid[:2] = grid[-2:] = grid[:, :2] = grid[:, -2:] = 0
        map_path = output/'map.pgm'
        map_path.write_bytes(b'P5\n160 160\n255\n'+grid.tobytes())
        (output/'map.yaml').write_text(yaml.safe_dump({
            'image': 'map.pgm', 'resolution': 0.05, 'origin': [-4., -4., 0.],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}))
    config = {
        'environment_id': 'labsea_fixed_fixture',
        'map_checksum': hashlib.sha256(map_path.read_bytes()).hexdigest(),
        'map_from_world': {'x': 0., 'y': 0., 'yaw': 0.},
        'ttl_sec': 3., 'association_radius_m': 0.8, 'fusion_window_sec': 0.2,
        'minimum_observations': 2, 'minimum_cameras': 2, 'maximum_sensitivity_m': 0.6,
        'planes': {'chair': {'z_m': 0.55, 'height_uncertainty_m': 0.25},
                   'dining table': {'z_m': 0.75, 'height_uncertainty_m': 0.08,
                                    'anchor_method': 'mask_incenter'}},
        'cameras': [{'id': i, 'calibration': 'calib/calib_rt'+i+'.npz',
                     'image_space': 'undistorted'} for i in ['1', '2', '3', '4']],
    }
    (output/'config.yaml').write_text(yaml.safe_dump(config))


def add_fixed_cameras(world, calibrations, update_rate=1.0):
    """Add calibrated full-frame RGB cameras to an existing Gazebo world."""
    matrices, bridges, calibration_hashes = {}, [], {}
    # Gazebo camera axes: forward X, left Y, up Z; optical axes: right X, down Y, forward Z.
    optical_from_gazebo = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
    for camera_id in ['1', '2', '3', '4']:
        path = calibrations/('calib_rt'+camera_id+'.npz')
        calibration_hashes[camera_id] = hashlib.sha256(path.read_bytes()).hexdigest()
        with np.load(path, allow_pickle=False) as calibration:
            k = calibration['nK'].copy()
            transform = calibration['rt_inv'].copy()
            width, height = int(calibration['width'].item()), int(calibration['height'].item())
        matrices[camera_id] = k
        model = ET.SubElement(world, 'model', name='fixed_camera_'+camera_id)
        add(model, 'static', 'true')
        angles = mat2euler(transform[:3, :3] @ optical_from_gazebo, axes='sxyz')
        add(model, 'pose', ' '.join(str(v) for v in [*transform[:3, 3], *angles]))
        link = ET.SubElement(model, 'link', name='camera')
        sensor = ET.SubElement(link, 'sensor', name='rgb', type='camera')
        prefix = '/fixed/camera_'+camera_id
        add(sensor, 'topic', prefix+'/image_raw')
        add(sensor, 'update_rate', update_rate)
        add(sensor, 'always_on', 'true')
        camera = ET.SubElement(sensor, 'camera')
        add(camera, 'camera_info_topic', prefix+'/camera_info')
        add(camera, 'horizontal_fov', 2*math.atan(width/(2*k[0, 0])))
        image = ET.SubElement(camera, 'image')
        add(image, 'width', width)
        add(image, 'height', height)
        add(image, 'format', 'R8G8B8')
        clip = ET.SubElement(camera, 'clip')
        add(clip, 'near', 0.05)
        add(clip, 'far', 30)
        lens = ET.SubElement(camera, 'lens')
        intrinsics = ET.SubElement(lens, 'intrinsics')
        for key, value in [('fx', k[0, 0]), ('fy', k[1, 1]), ('cx', k[0, 2]),
                           ('cy', k[1, 2]), ('s', 0.)]:
            add(intrinsics, key, value)
        for topic, ros_type, gz_type in [('image_raw', 'Image', 'Image'),
                                         ('camera_info', 'CameraInfo', 'CameraInfo')]:
            bridges.append({'ros_topic_name': prefix+'/'+topic, 'gz_topic_name': prefix+'/'+topic,
                            'ros_type_name': 'sensor_msgs/msg/'+ros_type,
                            'gz_type_name': 'gz.msgs.'+gz_type, 'direction': 'GZ_TO_ROS',
                            'publisher_queue': 1, 'subscriber_queue': 1})
    return matrices, bridges, calibration_hashes
