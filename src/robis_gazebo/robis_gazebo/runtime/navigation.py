#!/usr/bin/env python3
"""Map and exercise Nav2 on the known Gazebo world, without physical devices."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .gazebo import gazebo_pose


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def record_environment(directory, config_path):
    """Keep exact inputs and package versions beside the measured results."""
    root = Path(os.environ.get('ROBIS_WORKSPACE', '/robis_ws'))
    inputs = directory / 'inputs'
    inputs.mkdir(exist_ok=True)
    files = [Path(config_path),
        root / 'src/robis_navigation/config/nav2_params_demo.yaml',
        root / 'src/robis_navigation/config/slam_toolbox_demo.yaml',
        root / 'src/robis_description/config/robot_parameters.yaml',
        root / 'src/robis_description/urdf/robis.urdf.xacro',
        root / 'src/robis_gazebo/worlds/turtlebot3_like.world',
        root / 'extras/scripts/check_navigation.py']
    hashes = {}
    for source in files:
        shutil.copyfile(source, inputs / source.name)
        hashes[str(source.relative_to(root))] = hashlib.sha256(source.read_bytes()).hexdigest()
    packages = subprocess.run(['dpkg-query', '-W', 'ros-jazzy-navigation2',
        'ros-jazzy-slam-toolbox', 'ros-jazzy-ros-gz-sim',
        'ros-jazzy-nav2-regulated-pure-pursuit-controller'],
        check=True, capture_output=True, text=True, timeout=10).stdout
    metadata = {'recorded_utc': datetime.now(timezone.utc).isoformat(),
        'architecture': platform.machine(), 'kernel': platform.release(),
        'ros_distribution': os.environ.get('ROS_DISTRO'),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
        'rendering': 'CPU (LIBGL_ALWAYS_SOFTWARE=1)',
        'packages': dict(line.split('\t', 1) for line in packages.splitlines()),
        'input_sha256': hashes}
    (directory / 'environment.json').write_text(json.dumps(metadata, indent=2)+'\n')


class NavigationCheck(Node):
    def __init__(self, config):
        super().__init__('robis_navigation_check')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.config = config
        self.map = self.odom = self.scan = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.goal_publisher = self.create_publisher(PoseStamped, '/navigation_check/goal', 1)
        self.create_subscription(OccupancyGrid, '/map', self.receive_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/odom', lambda msg: setattr(self, 'odom', msg), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', lambda msg: setattr(self, 'scan', msg), qos_profile_sensor_data)
        self.navigate = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.plan = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.active_goal = None
        self.allow_motion = False
        self.trajectory = []
        self.record_trajectory = False
        self.last_sample = -1.0

    def receive_map(self, message):
        self.map = message

    def spin(self, seconds=0.02):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.001)

    def wait(self, predicate, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin()
            if self.record_trajectory:
                stamp = self.get_clock().now().nanoseconds / 1e9
                if stamp - self.last_sample >= 0.5:
                    self.trajectory.append(list(self.pose()))
                    self.last_sample = stamp
            if predicate():
                return
        raise RuntimeError(f'Tempo esgotado: {label}')

    def future(self, future, timeout, label):
        self.wait(future.done, timeout, label)
        result = future.result()
        if result is None:
            raise RuntimeError(f'Sem resposta: {label}')
        return result

    def pose(self):
        tf = self.buffer.lookup_transform('map', 'base_footprint', Time())
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        return t.x, t.y, yaw

    def ready(self):
        if self.map is None or self.odom is None or self.scan is None:
            return False
        if self.get_clock().now().nanoseconds <= 0:
            return False
        try:
            self.pose()
            self.buffer.lookup_transform('map', 'odom', Time())
        except TransformException:
            return False
        return True

    def guard(self):
        subscriptions = self.get_subscriptions_info_by_topic('/cmd_vel')
        names = {item.node_name for item in subscriptions}
        if 'cmd_vel_bridge' not in names or 'odrive_node' in self.get_node_names():
            raise RuntimeError('Teste exige Gazebo e recusa o driver ODrive no mesmo domínio')
        self.allow_motion = True

    def stop(self):
        if not self.allow_motion:
            return
        for _ in range(5):
            self.publisher.publish(Twist())
            self.spin()

    def settle(self):
        quiet_since = None
        def stopped():
            nonlocal quiet_since
            if self.odom is None:
                return False
            v = self.odom.twist.twist
            stamp = Time.from_msg(self.odom.header.stamp).nanoseconds
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if abs(v.linear.x) >= 0.01 or abs(v.angular.z) >= 0.02 or not 0 <= age < 1:
                quiet_since = None
                return False
            if quiet_since is None:
                quiet_since = time.monotonic()
            return time.monotonic() - quiet_since >= 0.5
        self.wait(stopped, 15, 'parada na odometria')
        self.spin(0.2)

    def map_world(self, directory):
        results = []
        for index, (gx, gy) in enumerate(self.config['mapping_waypoints'], 1):
            print(f'Mapeamento {index}: destino ({gx}, {gy})', flush=True)
            deadline = time.monotonic() + self.config['waypoint_timeout_wall_sec']
            while time.monotonic() < deadline:
                self.spin()
                x, y, yaw = self.pose()
                distance = math.hypot(gx-x, gy-y)
                if distance < self.config['waypoint_tolerance']:
                    break
                error = angle(math.atan2(gy-y, gx-x) - yaw)
                command = Twist()
                command.angular.z = max(-self.config['mapping_turn_speed'],
                    min(self.config['mapping_turn_speed'], 1.5*error))
                if abs(error) < 0.35:
                    command.linear.x = min(self.config['mapping_speed'], distance*0.8)
                self.publisher.publish(command)
            else:
                raise RuntimeError(f'Mapeamento não chegou ao waypoint {index}')
            self.stop()
            results.append({'goal': [gx, gy], 'pose': list(self.pose())})
        # Return to the startup orientation, then save the finished map.
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            self.spin()
            error = angle(-self.pose()[2])
            if abs(error) < 0.05:
                break
            command = Twist()
            command.angular.z = max(-0.4, min(0.4, 1.5*error))
            self.publisher.publish(command)
        else:
            raise RuntimeError('Mapeamento não retornou à orientação inicial')
        self.stop()
        self.settle()
        self.spin(5)
        client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Serviço de salvar mapa indisponível')
        request = SaveMap.Request()
        request.name.data = str(directory / 'map')
        response = self.future(client.call_async(request), 20, 'salvar mapa')
        if response.result != SaveMap.Response.RESULT_SUCCESS:
            raise RuntimeError(f'Falha ao salvar mapa: {response.result}')
        for extension in ('yaml', 'pgm'):
            if not (directory / f'map.{extension}').is_file():
                raise RuntimeError(f'Arquivo map.{extension} não foi salvo')
        known = sum(value >= 0 for value in self.map.data)
        occupied = sum(value >= 65 for value in self.map.data)
        if known < 1000 or occupied < 100:
            raise RuntimeError('Mapa gerado não contém área livre e obstáculos suficientes')
        return {'success': True, 'waypoints': results, 'map': {
            'width': self.map.info.width, 'height': self.map.info.height,
            'resolution': self.map.info.resolution, 'known_cells': known,
            'occupied_cells': occupied, 'yaml': str(directory / 'map.yaml')},
            'final_pose': list(self.pose())}

    def lifecycle_active(self, name):
        client = self.create_client(GetState, f'/{name}/get_state')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f'Nó {name} indisponível')
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            response = self.future(client.call_async(GetState.Request()), 10, name)
            if response.current_state.id == 3:
                self.destroy_client(client)
                return
            self.spin(0.2)
        raise RuntimeError(f'Nó {name} não ativou')

    def goal_pose(self, values):
        x, y, yaw = values
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        return pose

    def calculate_path(self, values):
        goal = ComputePathToPose.Goal()
        goal.goal = self.goal_pose(values)
        goal.planner_id = 'GridBased'
        handle = self.future(self.plan.send_goal_async(goal), 15, 'aceitar planejamento')
        if not handle.accepted:
            raise RuntimeError('Planejador recusou a solicitação')
        return self.future(handle.get_result_async(), 30, 'calcular caminho')

    def navigation(self):
        for name in ['map_server', 'amcl', 'controller_server', 'planner_server',
                     'smoother_server', 'behavior_server', 'bt_navigator',
                     'waypoint_follower', 'velocity_smoother']:
            self.lifecycle_active(name)
        if not self.navigate.wait_for_server(timeout_sec=10) or not self.plan.wait_for_server(timeout_sec=10):
            raise RuntimeError('Servidores de ações do Nav2 indisponíveis')
        self.settle()
        results = []
        for index, goal_values in enumerate(self.config['navigation_goals'], 1):
            print(f'Nav2 {index}: destino {goal_values}', flush=True)
            start = self.pose()
            path = self.calculate_path(goal_values)
            if path.status != GoalStatus.STATUS_SUCCEEDED or path.result.error_code:
                raise RuntimeError(f'Planejamento falhou: {path.result.error_msg}')
            points = path.result.path.poses
            length = sum(math.hypot(b.pose.position.x-a.pose.position.x,
                                   b.pose.position.y-a.pose.position.y) for a, b in zip(points, points[1:]))
            direct = math.hypot(start[0]-goal_values[0], start[1]-goal_values[1])
            if index == 1 and length < direct*self.config['first_goal_min_detour_ratio']:
                raise RuntimeError('O primeiro caminho não contornou a parede do cenário')
            goal = NavigateToPose.Goal()
            goal.pose = self.goal_pose(goal_values)
            self.goal_publisher.publish(goal.pose)
            self.active_goal = self.future(self.navigate.send_goal_async(goal), 15, 'aceitar destino')
            if not self.active_goal.accepted:
                raise RuntimeError('Nav2 recusou o destino')
            started = time.monotonic()
            self.trajectory = []
            self.record_trajectory = True
            outcome = self.future(self.active_goal.get_result_async(),
                self.config['goal_timeout_wall_sec'], 'chegar ao destino')
            self.record_trajectory = False
            self.active_goal = None
            if outcome.status != GoalStatus.STATUS_SUCCEEDED or outcome.result.error_code:
                raise RuntimeError(f'Navegação falhou: {outcome.status}, {outcome.result.error_msg}')
            self.settle()
            pose = self.pose()
            native = gazebo_pose(self.config['world'], self.config['robot'])
            point = native['position']
            map_error = math.hypot(pose[0]-goal_values[0], pose[1]-goal_values[1])
            native_error = math.hypot(point.get('x', 0)-goal_values[0], point.get('y', 0)-goal_values[1])
            if max(map_error, native_error) > self.config['goal_tolerance']:
                raise RuntimeError(f'Chegada fora da tolerância: mapa={map_error:.3f}, Gazebo={native_error:.3f}')
            results.append({'goal': goal_values, 'status': outcome.status, 'pose': list(pose),
                'map_error_m': map_error, 'gazebo_error_m': native_error,
                'gazebo_position': point,
                'path_length_m': length, 'direct_distance_m': direct,
                'planned_path': [[p.pose.position.x, p.pose.position.y] for p in points],
                'trajectory': self.trajectory,
                'wall_seconds': time.monotonic()-started})
            print(f'Chegada {index}: erro Gazebo {native_error:.3f} m, caminho {length:.2f} m', flush=True)
        # Cancel a moving action and observe the stop produced by Nav2 itself.
        start = self.pose()
        goal = NavigateToPose.Goal()
        goal.pose = self.goal_pose(self.config['navigation_goals'][0])
        self.active_goal = self.future(self.navigate.send_goal_async(goal), 15, 'aceitar ação para cancelamento')
        if not self.active_goal.accepted:
            raise RuntimeError('Nav2 recusou a ação de cancelamento')
        result_future = self.active_goal.get_result_async()
        self.wait(lambda: math.hypot(self.pose()[0]-start[0], self.pose()[1]-start[1]) > 0.15,
                  60, 'movimento antes do cancelamento')
        response = self.future(self.active_goal.cancel_goal_async(), 10, 'cancelar movimento')
        outcome = self.future(result_future, 15, 'encerrar ação cancelada')
        self.active_goal = None
        if response.return_code != 0 or outcome.status != GoalStatus.STATUS_CANCELED:
            raise RuntimeError('Cancelamento não foi confirmado pelo Nav2')
        self.settle()
        invalid = self.calculate_path(self.config['outside_map_goal'])
        if invalid.status == GoalStatus.STATUS_SUCCEEDED or invalid.result.error_code == 0:
            raise RuntimeError('Destino fora do mapa foi aceito como válido')
        return {'success': True, 'goals': results,
            'outside_map_error_code': invalid.result.error_code, 'stopped': True,
            'cancellation_status': outcome.status,
            'tf_chain': ['map', 'odom', 'base_footprint']}
