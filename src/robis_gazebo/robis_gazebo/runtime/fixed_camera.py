#!/usr/bin/env python3
"""Ceiling RGB seeds empty memory; real Nav2 actions end with onboard RGB/LiDAR confirmation.

Only run in the dedicated Gazebo container. No physical devices are needed.
Reference object poses are read exclusively after each mission for evaluation.
"""
import argparse
from contextlib import ExitStack, contextmanager
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import cv2
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import yaml

from robis_interfaces.action import GotoObject
from robis_semantic_navigation.store import SemanticStore
from .fixed_camera_setup import add_fixed_cameras, write_fixture_configuration
from .gazebo import gazebo_pose
from .semantic import SemanticCheck


WORLD = 'labsea_fixed_mission'
ENVIRONMENT = 'labsea_fixed_fixture'


class Process:
    def __init__(self, command, path):
        self.log = path.open('w')
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)

    def close(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                try:
                    self.process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=5)
        self.log.close()


def start(stack, command, path):
    process = Process(command, path)
    stack.callback(process.close)
    return process


def ros_node(stack, kind=SemanticCheck):
    rclpy.init(args=[])
    stack.callback(lambda: rclpy.shutdown() if rclpy.ok() else None)
    node = kind(WORLD, ENVIRONMENT)
    stack.callback(node.destroy_node)
    return node


def mapping(root, world_path):
    with ExitStack() as stack:
        start(stack, ['ros2', 'launch', 'robis_gazebo', 'simulation.launch.py',
                      'world:='+str(world_path), 'enable_ground_truth_sensors:=false',
                      'gz_args:=-s -r --headless-rendering -v 2'], root/'mapping-simulation.log')
        start(stack, ['ros2', 'launch', 'robis_navigation', 'slam.launch.py',
                      'params_file:='+str(Path(get_package_share_directory('robis_navigation')) /
                                          'config/slam_toolbox_demo.yaml'),
                      'use_sim_time:=true', 'rviz:=false'], root/'mapping-slam.log')
        node = ros_node(stack)
        node.wait(node.ready, 120, 'SLAM, laser, odometria e TF sem mover o robô')
        node.spin(5)
        checksum = node.save_map(root)
        result = {'source': 'stationary simulated planar lidar and SLAM',
                  'checksum': checksum, 'map_pose': list(node.pose()),
                  'width': node.map.info.width, 'height': node.map.info.height,
                  'no_semantic_memory_created': not (root/'objects.yaml').exists()}
        assert result['no_semantic_memory_created']
        return result


class MissionCheck(SemanticCheck):
    def __init__(self, world, environment):
        super().__init__(world, environment)
        self.image = self.debug = self.ceiling_status = None
        self.feedback_events = []
        self.create_subscription(Image, '/camera/image_raw', lambda m: setattr(self, 'image', m), 1)
        self.create_subscription(Image, '/semantic_perception/debug_image',
                                 lambda m: setattr(self, 'debug', m), 1)
        self.create_subscription(String, '/semantic_navigation/fixed_camera_status',
                                 lambda m: setattr(self, 'ceiling_status', json.loads(m.data)), 10)

    def feedback(self, message):
        super().feedback(message)
        self.feedback_events.append({'state': message.feedback.state,
                                     'description': message.feedback.description,
                                     'ros_stamp': self.get_clock().now().nanoseconds/1e9})


@contextmanager
def scene_session(root, world_path, bridge_path, config_path, model, checksum, report, spawn=(0., 0., 0.)):
    memory = root/'objects.yaml'
    nav_config = yaml.safe_load((Path(get_package_share_directory('robis_navigation')) /
                                'config/nav2_params_demo.yaml').read_text())
    nav_config['amcl']['ros__parameters']['initial_pose'].update(x=spawn[0], y=spawn[1], yaw=spawn[2])
    # Semantic navigation already chooses free approach candidates. Do not silently
    # relocate an occupied coordinate goal by the NavFn default half-metre tolerance.
    nav_config['planner_server']['ros__parameters']['GridBased']['tolerance'] = 0.0
    nav_path = root/'navigation-params.yaml'
    nav_path.write_text(yaml.safe_dump(nav_config))
    with ExitStack() as stack:
        processes = []
        processes.append(start(stack, ['ros2', 'launch', 'robis_bringup', 'semantic_simulation.launch.py',
                      'mode:=navigation', 'world:='+str(world_path), 'map:='+str(root/'map.yaml'),
                      'nav_params_file:='+str(nav_path),
                      'spawn_x:='+str(spawn[0]), 'spawn_y:='+str(spawn[1]), 'spawn_yaw:='+str(spawn[2]),
                      'semantic_map:='+str(memory), 'map_checksum:='+checksum,
                      'detector_backend:=yolo26', 'detector_model:='+str(model), 'detector_device:=0',
                      'detector_image_size:=640', 'detector_inference_hz:=5.0',
                      'range_estimator:=mask_projected_cluster', 'mask_pixel_tolerance_px:=2',
                      'enable_ground_truth_sensors:=false', 'use_depth_hint:=false',
                      'enable_external_memory:=true', 'external_environment_id:='+ENVIRONMENT,
                      'external_max_age_sec:=3.0', 'rviz:=false',
                      'gz_args:=-s -r --headless-rendering -v 2'], root/'navigation-stack.log'))
        processes.append(start(stack, ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '--ros-args',
                      '-p', 'config_file:='+str(bridge_path), '-p', 'use_sim_time:=true'],
              root/'ceiling-bridge.log'))
        node = ros_node(stack, MissionCheck)
        node.wait(lambda: node.ready() and node.perception and node.image is not None,
                  120, 'AMCL, Nav2 e percepção embarcada')
        node.guard()
        stack.callback(node.stop)
        names = set(node.get_node_names())
        assert not names.intersection({'oracle_detector', 'ground_truth_bridge', 'odrive_node'})
        assert 'yolo26_detector' in names
        for name in ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                     'bt_navigator', 'waypoint_follower', 'velocity_smoother']:
            node.lifecycle_active(name)
        assert node.semantic.wait_for_server(timeout_sec=10)
        report['initial_memory_empty'] = not memory.exists()
        report['initial_map_pose'] = list(node.pose())
        ceiling = start(stack, ['ros2', 'run', 'robis_semantic_navigation', 'fixed_camera_stream',
                                '--ros-args', '-p', 'use_sim_time:=true',
                                '-p', 'configuration:='+str(config_path), '-p', 'model:='+str(model),
                                '-p', "device:='0'", '-p', 'image_size:=1280',
                                '-p', 'confidence:=0.4', '-p', 'exclude_robot:=true',
                                '-p', 'output_directory:='+str(root/'ceiling')], root/'ceiling-stream.log')

        def instances(label):
            return SemanticStore(memory).instances(ENVIRONMENT, label, map_checksum=checksum,
                                                   ros_now_ns=node.get_clock().now().nanoseconds)

        def external_ready():
            if ceiling.process.poll() is not None:
                raise RuntimeError('Ceiling process exited; inspect ceiling-stream.log')
            return len(instances('chair')) >= 1 and len(instances('dining table')) == 1

        node.wait(external_ready, 90, 'cadeira e mesa confirmadas pelas câmeras fixas')
        processes.append(ceiling)
        yield node, ceiling, processes
