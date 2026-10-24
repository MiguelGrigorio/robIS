#!/usr/bin/env python3
"""Exercise real semantic ROS actions in Gazebo, using labels and planar lidar."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
from slam_toolbox.srv import SaveMap
import rclpy
from robis_interfaces.action import GotoObject
import yaml

from .navigation import NavigationCheck
from .gazebo import gazebo_pose


def snapshot(directory):
    root = Path('/robis_ws')
    inputs = directory / 'inputs'
    inputs.mkdir(exist_ok=True)
    files = ['src/robis_semantic_navigation/config/semantic_navigation.yaml',
        'src/robis_semantic_navigation/config/aliases.yaml',
        'src/robis_semantic_navigation/robis_semantic_navigation/server.py',
        'src/robis_semantic_navigation/robis_semantic_navigation/object_localizer.py',
        'src/robis_vision/config/oracle_detector.yaml',
        'src/robis_vision/robis_vision/oracle_detector.py',
        'src/robis_navigation/config/nav2_params_demo.yaml',
        'src/robis_description/config/robot_parameters.yaml',
        'src/robis_gazebo/worlds/semantic_lab.world', 'extras/scripts/check_semantic.py']
    hashes = {}
    for name in files:
        source = root / name
        shutil.copyfile(source, inputs / source.name)
        hashes[name] = hashlib.sha256(source.read_bytes()).hexdigest()
    versions = subprocess.run(['dpkg-query', '-W', 'ros-jazzy-navigation2',
        'ros-jazzy-slam-toolbox', 'ros-jazzy-ros-gz-sim'],
        check=True, capture_output=True, text=True, timeout=10).stdout
    value = {'recorded_utc': datetime.now(timezone.utc).isoformat(),
        'packages': dict(line.split('\t', 1) for line in versions.splitlines()),
        'input_sha256': hashes}
    (directory / 'environment.json').write_text(json.dumps(value, indent=2)+'\n')


class SemanticCheck(NavigationCheck):
    def __init__(self, world='semantic_lab', environment_id='semantic_lab_check'):
        super().__init__({'world': world, 'robot': 'robis'})
        self.environment_id = environment_id
        self.semantic = ActionClient(self, GotoObject, '/goto_object')
        self.perception = False
        self.sources = Counter()
        self.states = []
        self.create_subscription(Bool, '/semantic_navigation/perception_ready',
            lambda msg: setattr(self, 'perception', msg.data), 10)
        self.create_subscription(String, '/semantic_navigation/localization_diagnostics',
            self.diagnostic, 50)
        self.active_semantic = None

    def diagnostic(self, message):
        item = json.loads(message.data)
        if item['localized']:
            self.sources[item['source']] += 1

    def feedback(self, message):
        item = message.feedback
        if not self.states or self.states[-1] != item.state:
            self.states.append(item.state)
            print(f'Semântica: {item.description}', flush=True)

    def mission(self, query, cancel=False, timeout=150.0):
        self.states = []
        goal = GotoObject.Goal()
        goal.query = query
        goal.environment_id = self.environment_id
        goal.timeout_sec = float(timeout)
        self.active_semantic = self.future(self.semantic.send_goal_async(
            goal, feedback_callback=self.feedback), 15, 'aceitar consulta semântica')
        if not self.active_semantic.accepted:
            raise RuntimeError('Consulta foi recusada antes de executar')
        result_future = self.active_semantic.get_result_async()
        cancel_velocity = None
        if cancel:
            def moving():
                velocity = self.odom.twist.twist
                return abs(velocity.linear.x) > 0.02 or abs(velocity.angular.z) > 0.04
            self.wait(moving, 45, 'movimento durante busca do objeto ausente')
            velocity = self.odom.twist.twist
            cancel_velocity = [velocity.linear.x, velocity.angular.z]
            response = self.future(self.active_semantic.cancel_goal_async(), 10, 'cancelar consulta')
            if response.return_code:
                raise RuntimeError('Cancelamento semântico recusado')
        outcome = self.future(result_future, timeout+15 if not cancel else 20, 'resultado semântico')
        self.active_semantic = None
        self.settle()
        result = outcome.result
        point = result.object_pose.pose.position
        return {'query': query, 'status': outcome.status, 'success': result.success,
            'error_code': result.error_code, 'message': result.message,
            'states': list(self.states), 'elapsed_wall_seconds': result.elapsed_sec,
            'velocity_at_cancel': cancel_velocity,
            'odometry_distance_m': result.distance_traveled_m,
            'time_to_first_detection_wall_seconds': result.time_to_first_detection_sec,
            'object_xy': [point.x, point.y], 'robot_map_pose': list(self.pose())}

    def success(self, result):
        if result['status'] != GoalStatus.STATUS_SUCCEEDED or not result['success']:
            raise RuntimeError(f'Missão falhou: {result}')
        native = gazebo_pose('semantic_lab', 'robis')['position']
        center = (2.7, 1.7)
        distance = ((native.get('x', 0)-center[0])**2 + (native.get('y', 0)-center[1])**2)**0.5
        error = ((result['object_xy'][0]-center[0])**2 + (result['object_xy'][1]-center[1])**2)**0.5
        if not 1.05 <= distance <= 2.3 or error > 0.7:
            raise RuntimeError(f'Aproximação física/localização inválida: distância={distance}, erro={error}')
        result.update(gazebo_robot_position=native, distance_to_chair_center_m=distance,
            estimated_point_to_chair_center_m=error)

    def save_map(self, directory):
        client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Serviço para salvar mapa indisponível')
        request = SaveMap.Request()
        request.name.data = str(directory / 'map')
        result = self.future(client.call_async(request), 20, 'salvar mapa de navegação')
        if result.result != SaveMap.Response.RESULT_SUCCESS:
            raise RuntimeError('Mapa não foi salvo')
        return hashlib.sha256((directory / 'map.pgm').read_bytes()).hexdigest()
