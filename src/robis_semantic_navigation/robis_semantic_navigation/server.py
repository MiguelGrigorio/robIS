#!/usr/bin/env python3
"""Action server orchestrating memory, Nav2, search and reconfirmation."""

import math
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray
import yaml

from robis_interfaces.action import GotoObject
from robis_interfaces.msg import ExternalObjects
from robis_interfaces.srv import DetectObject

from .geometry import (
    approach_candidates,
    nearest_neighbor_route,
    occupancy_coverage_waypoints,
    within_standoff,
)
from .naming import normalize_query, requires_vlm
from .live_target import LiveTarget, TargetLost, TargetMoved
from .observations import ObservationCache
from .store import SemanticStore


class SemanticNavigationServer(Node):
    def __init__(self):
        super().__init__("semantic_navigation_server")
        default_config = Path(get_package_share_directory("robis_semantic_navigation")) / "config"
        self.declare_parameter("semantic_map", "/robis_ws/data/environments/semantic_map.yaml")
        self.declare_parameter("waypoints_file", str(default_config / "waypoints.yaml"))
        self.declare_parameter("aliases_file", str(default_config / "aliases.yaml"))
        self.declare_parameter("objects_topic", "/semantic_navigation/objects")
        self.declare_parameter(
            "perception_ready_topic", "/semantic_navigation/perception_ready"
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("maximum_odom_step", 0.5)
        self.declare_parameter("search_mode", "auto")
        self.declare_parameter("coverage_spacing", 1.5)
        self.declare_parameter("coverage_clearance", 0.45)
        self.declare_parameter("coverage_occupancy_threshold", 20)
        self.declare_parameter("coverage_max_waypoints", 40)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("instance_policy", "nearest")
        self.declare_parameter("default_timeout_sec", 300.0)
        self.declare_parameter("observation_max_age_sec", 1.0)
        self.declare_parameter("perception_status_max_age_sec", 2.0)
        self.declare_parameter("sensor_startup_timeout_sec", 8.0)
        self.declare_parameter("confirmation_count", 3)
        self.declare_parameter("confirmation_radius", 0.8)
        self.declare_parameter("confirmation_timeout_sec", 6.0)
        self.declare_parameter("standoff_distance", 1.4)
        self.declare_parameter("standoff_tolerance", 0.35)
        self.declare_parameter(
            "approach_angle_offsets_deg",
            [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0],
        )
        self.declare_parameter("planner_service_timeout_sec", 2.0)
        self.declare_parameter("merge_distance", 0.5)
        self.declare_parameter("spin_segments", 4)
        self.declare_parameter("map_checksum", "")
        self.declare_parameter("enable_external_memory", False)
        self.declare_parameter("external_objects_topic", "/semantic_navigation/external_objects")
        self.declare_parameter("external_environment_id", "")
        self.declare_parameter("external_max_age_sec", 3.0)
        self.declare_parameter("external_max_sensitivity_m", 0.6)
        self.declare_parameter("live_external_targets", True)
        self.declare_parameter("target_replan_distance_m", 0.35)
        self.declare_parameter("target_max_replans", 10)
        if (not math.isfinite(self.get_parameter('target_replan_distance_m').value)
                or self.get_parameter('target_replan_distance_m').value <= 0
                or self.get_parameter('target_max_replans').value < 1):
            raise ValueError('Live targets require positive replan distance and count')
        if self.get_parameter("enable_external_memory").value and (
                not self.get_parameter("external_environment_id").value
                or len(self.get_parameter("map_checksum").value) != 64
                or any(c not in '0123456789abcdef' for c in self.get_parameter("map_checksum").value)
                or not math.isfinite(self.get_parameter("external_max_age_sec").value)
                or self.get_parameter("external_max_age_sec").value <= 0
                or not math.isfinite(self.get_parameter("external_max_sensitivity_m").value)
                or self.get_parameter("external_max_sensitivity_m").value <= 0):
            raise ValueError("External memory requires environment, map checksum and positive max age")
        self.declare_parameter("vlm_retry_interval_sec", 10.0)
        self.group = ReentrantCallbackGroup()
        self.store = SemanticStore(self.get_parameter("semantic_map").value)
        self.aliases = self._read_yaml(self.get_parameter("aliases_file").value).get("aliases", {})
        self.waypoints = self._read_yaml(self.get_parameter("waypoints_file").value).get("environments", {})
        self.observations = ObservationCache()
        self.perception_ready = False
        self.perception_status_time = 0.0
        self.vlm_retry_after = 0.0
        self.occupancy_map = None
        self._last_odom_xy = None
        self._odometry_distance_total = 0.0
        self._mission_started = None
        self._mission_distance_start = 0.0
        self._mission_points_visited = 0
        self._mission_first_detection_sec = -1.0
        self._goal_state_lock = threading.Lock()
        self._goal_active = False
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self.group)
        self.spin_client = ActionClient(self, Spin, "spin", callback_group=self.group)
        self.path_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose", callback_group=self.group
        )
        self.vlm_client = self.create_client(
            DetectObject, "/semantic_perception/detect_object_vlm", callback_group=self.group
        )
        self.active_nav_goal = None
        self._live_target = None
        self.create_subscription(
            Detection3DArray, self.get_parameter("objects_topic").value,
            self._objects_callback, 10, callback_group=self.group
        )
        if self.get_parameter("enable_external_memory").value:
            self.create_subscription(
                ExternalObjects, self.get_parameter("external_objects_topic").value,
                self._external_objects_callback, 10, callback_group=self.group)
        self.create_subscription(
            Bool, self.get_parameter("perception_ready_topic").value,
            self._perception_status_callback, 10, callback_group=self.group
        )
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value,
            self._map_callback, map_qos, callback_group=self.group
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self._odom_callback, 20, callback_group=self.group
        )
        self.server = ActionServer(
            self, GotoObject, "goto_object", execute_callback=self._execute,
            goal_callback=self._goal_callback, cancel_callback=self._cancel_callback,
            callback_group=self.group,
        )

    @staticmethod
    def _read_yaml(filename):
        try:
            with open(filename, "r", encoding="utf-8") as stream:
                return yaml.safe_load(stream) or {}
        except OSError:
            return {}

    def _objects_callback(self, message):
        frame = self.get_parameter("map_frame").value
        if message.header.frame_id != frame:
            return
        grouped = {}
        for detection in message.detections:
            if not detection.results:
                continue
            if detection.header.frame_id and detection.header.frame_id != frame:
                continue
            point = detection.bbox.center.position
            score = float(detection.results[0].hypothesis.score)
            if not all(math.isfinite(v) for v in (point.x, point.y, point.z, score)):
                continue
            if not 0.0 <= score <= 1.0:
                continue
            label = normalize_query(
                detection.results[0].hypothesis.class_id, self.aliases
            )
            grouped.setdefault(label, []).append(detection)
        self.observations.update(grouped)

    def _perception_status_callback(self, message):
        self.perception_ready = bool(message.data)
        self.perception_status_time = time.monotonic()

    def _external_objects_callback(self, message):
        """Import calibrated environment observations through the sole YAML writer."""
        if not self.get_parameter("enable_external_memory").value:
            return
        if (message.header.frame_id != self.get_parameter("map_frame").value
                or message.environment_id != self.get_parameter("external_environment_id").value
                or message.map_checksum != self.get_parameter("map_checksum").value
                or not message.source_id or len(message.source_id) > 128
                or not math.isfinite(message.validity_sec) or message.validity_sec <= 0):
            return
        ttl = min(message.validity_sec, self.get_parameter("external_max_age_sec").value)
        now_ns = self.get_clock().now().nanoseconds
        array_stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if not -0.05 <= (now_ns - array_stamp) / 1e9 < ttl:
            return
        for observation in message.objects:
            detection = observation.detection
            if (not detection.id or len(detection.id) > 128 or not detection.results
                    or not math.isfinite(observation.projection_sensitivity_m)
                    or not 0 <= observation.projection_sensitivity_m <= self.get_parameter(
                        "external_max_sensitivity_m").value
                    or detection.header.frame_id != message.header.frame_id):
                continue
            stamp_ns = detection.header.stamp.sec * 10**9 + detection.header.stamp.nanosec
            age = (now_ns - stamp_ns) / 1e9
            point = detection.bbox.center.position
            result = detection.results[0].hypothesis
            label = normalize_query(result.class_id, self.aliases)
            if (not label or not -0.05 <= age < ttl
                    or not all(math.isfinite(v) for v in (point.x, point.y, point.z, result.score))
                    or not 0 <= result.score <= 1):
                continue
            try:
                self.store.upsert_external(
                    message.environment_id, label, message.source_id + ':' + detection.id,
                    {'x': point.x, 'y': point.y, 'yaw': 0.0}, result.score,
                    stamp_ns, time.time() + ttl - max(0., age), message.map_checksum,
                    revision=observation.revision, validity_sec=ttl)
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=5.0)

    def _map_callback(self, message):
        self.occupancy_map = message

    def _odom_callback(self, message):
        point = message.pose.pose.position
        current = (float(point.x), float(point.y))
        if self._last_odom_xy is not None:
            step = math.hypot(
                current[0] - self._last_odom_xy[0],
                current[1] - self._last_odom_xy[1],
            )
            if step <= float(self.get_parameter("maximum_odom_step").value):
                self._odometry_distance_total += step
        self._last_odom_xy = current

    def _goal_callback(self, goal):
        with self._goal_state_lock:
            if self._goal_active:
                self.get_logger().warning("Rejeitando meta: outra busca já está ativa")
                return GoalResponse.REJECT
            self._goal_active = True
            return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal):
        return CancelResponse.ACCEPT

    def _feedback(self, handle, state, description, started, waypoint=-1, confidence=0.0):
        feedback = GotoObject.Feedback()
        feedback.state = state
        feedback.description = description
        feedback.waypoint_index = waypoint
        feedback.elapsed_sec = float(time.monotonic() - started)
        feedback.best_confidence = float(confidence)
        handle.publish_feedback(feedback)
        self.get_logger().info(description)

    def _robot_xy(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter("map_frame").value,
                self.get_parameter("base_frame").value, rclpy.time.Time(),
                timeout=Duration(seconds=0.3)
            )
            return transform.transform.translation.x, transform.transform.translation.y
        except TransformException as error:
            self.get_logger().warning(
                f"Pose do robô indisponível: {error}", throttle_duration_sec=5.0
            )
            return None

    @staticmethod
    def _pose(x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _matching_observation(self, label, expected=None):
        if self._live_target is not None:
            # Keep the same CCTV identity throughout approach and confirmation.
            expected = self._current_live_target()
            if expected is None:
                return None
        snapshot = self.observations.snapshot(
            label, float(self.get_parameter("observation_max_age_sec").value)
        )
        if snapshot is None:
            return None
        sequence, detections = snapshot
        candidates = []
        for detection in detections:
            if not detection.results:
                continue
            result = detection.results[0]
            if normalize_query(result.hypothesis.class_id, self.aliases) != label:
                continue
            point = detection.bbox.center.position
            if expected is not None:
                radius = float(self.get_parameter("confirmation_radius").value)
                if math.hypot(point.x - expected["pose"]["x"], point.y - expected["pose"]["y"]) > radius:
                    continue
            candidates.append((float(result.hypothesis.score), point))
        if not candidates:
            return None
        if expected is not None:
            selected = max(candidates, key=lambda item: item[0])
        elif self.get_parameter("instance_policy").value == "nearest":
            robot_xy = self._robot_xy()
            if robot_xy is None:
                return None
            selected = min(
                candidates,
                key=lambda item: math.hypot(
                    item[1].x - robot_xy[0], item[1].y - robot_xy[1]
                ),
            )
        else:
            selected = max(candidates, key=lambda item: item[0])
        if self._mission_started is not None and self._mission_first_detection_sec < 0:
            self._mission_first_detection_sec = time.monotonic() - self._mission_started
        return sequence, selected

    def _waypoint_route(self, environment):
        """Resolve an exact route or a ``route:run_id`` campaign environment."""
        if environment in self.waypoints:
            route = self.waypoints[environment]
        else:
            route_name = environment.split(":", 1)[0]
            route = self.waypoints.get(
                route_name, self.waypoints.get("default", [])
            )
        robot_xy = self._robot_xy()
        return nearest_neighbor_route(route, robot_xy) if robot_xy else route

    def _coverage_route(self):
        message = self.occupancy_map
        if message is None:
            return []
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        route = occupancy_coverage_waypoints(
            message.data,
            message.info.width,
            message.info.height,
            message.info.resolution,
            message.info.origin.position.x,
            message.info.origin.position.y,
            yaw,
            float(self.get_parameter("coverage_spacing").value),
            float(self.get_parameter("coverage_clearance").value),
            int(self.get_parameter("coverage_occupancy_threshold").value),
            int(self.get_parameter("coverage_max_waypoints").value),
        )
        robot_xy = self._robot_xy()
        return nearest_neighbor_route(route, robot_xy) if robot_xy else route

    def _search_route(self, environment):
        mode = str(self.get_parameter("search_mode").value).lower()
        if mode not in ("auto", "coverage", "waypoints"):
            self.get_logger().warning(
                f"search_mode={mode!r} inválido; usando auto"
            )
            mode = "auto"
        if mode in ("auto", "coverage"):
            route = self._coverage_route()
            if route:
                return route, "cobertura automática"
            if mode == "coverage":
                return [], "cobertura automática"
            self.get_logger().warning(
                "Mapa de ocupação sem pontos seguros; usando waypoints configurados"
            )
        return self._waypoint_route(environment), "waypoints configurados"

    async def _wait_servers(self, handle, deadline):
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                return False
            if (self.nav_client.server_is_ready()
                    and self.spin_client.server_is_ready()
                    and self.path_client.server_is_ready()):
                return True
            time.sleep(0.2)
        return False

    async def _wait_perception(self, handle, deadline):
        status_max_age = float(
            self.get_parameter("perception_status_max_age_sec").value
        )
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                return False
            status_fresh = (
                self.perception_status_time > 0.0
                and time.monotonic() - self.perception_status_time <= status_max_age
            )
            if status_fresh and self.perception_ready:
                return True
            time.sleep(0.2)
        return False

    @staticmethod
    def _path_length(path):
        return sum(
            math.hypot(
                current.pose.position.x - previous.pose.position.x,
                current.pose.position.y - previous.pose.position.y,
            )
            for previous, current in zip(path.poses, path.poses[1:])
        )

    async def _cancel_subgoal(self, goal=None):
        target = goal or self.active_nav_goal
        if target is None:
            return
        try:
            await target.cancel_goal_async()
        except Exception as error:
            self.get_logger().warning(f"Falha ao cancelar submeta Nav2: {error}")
        finally:
            if self.active_nav_goal is target:
                self.active_nav_goal = None

    def _cancel_late_goal(self, future):
        """Cancel the remote goal even when its acceptance arrives after timeout."""
        def accepted_later(completed):
            try:
                goal = completed.result()
                if goal is not None and goal.accepted:
                    goal.cancel_goal_async()
            except Exception as error:
                self.get_logger().warning(f"Falha ao cancelar aceitação tardia: {error}")
        future.add_done_callback(accepted_later)

    def _current_live_target(self):
        target = self._live_target
        if target is None:
            return None
        return next((item for item in self.store.instances(
            self._live_environment, self._live_label,
            ros_now_ns=self.get_clock().now().nanoseconds)
            if item['id'] == target.instance['id']), None)

    def _check_live_target(self):
        if self._live_target is not None:
            return self._live_target.check(self._current_live_target())
        return None

    async def _guard_live_goal(self, goal, result_future):
        try:
            self._check_live_target()
        except (TargetMoved, TargetLost):
            await self._cancel_subgoal(goal)
            # Cancellation acknowledgement alone is not proof that the old
            # controller finished. Wait for its terminal action result.
            stop = time.monotonic() + 3.0
            while not result_future.done() and time.monotonic() < stop:
                time.sleep(0.05)
            if not result_future.done():
                raise TargetLost('Nav2 não confirmou parada da meta anterior')
            raise

    async def _compute_path(self, pose, handle, deadline):
        if handle.is_cancel_requested or time.monotonic() >= deadline:
            return None
        request = ComputePathToPose.Goal()
        request.goal = pose
        request.use_start = False
        goal_future = self.path_client.send_goal_async(request)
        stop = min(
            deadline,
            time.monotonic()
            + float(self.get_parameter("planner_service_timeout_sec").value),
        )
        while not goal_future.done() and time.monotonic() < stop:
            if handle.is_cancel_requested:
                self._cancel_late_goal(goal_future)
                return None
            time.sleep(0.05)
        if not goal_future.done():
            self._cancel_late_goal(goal_future)
            return None
        try:
            goal = goal_future.result()
            if not goal.accepted:
                return None
            self.active_nav_goal = goal
            result_future = goal.get_result_async()
            while not result_future.done() and time.monotonic() < stop:
                if handle.is_cancel_requested:
                    await self._cancel_subgoal(goal)
                    return None
                time.sleep(0.05)
            if not result_future.done():
                await self._cancel_subgoal(goal)
                return None
            outcome = result_future.result()
            if outcome.status != GoalStatus.STATUS_SUCCEEDED or outcome.result.error_code:
                return None
            response = outcome.result
        except Exception as error:
            self.get_logger().warning(f"Falha ao validar pose de aproximação: {error}")
            return None
        finally:
            if "goal" in locals() and self.active_nav_goal is goal:
                self.active_nav_goal = None
        if response is None or not response.path.poses:
            return None
        return self._path_length(response.path)

    async def _select_approach_pose(self, object_x, object_y, handle, deadline):
        self._check_live_target()
        robot_xy = self._robot_xy()
        if robot_xy is None:
            return None
        robot_x, robot_y = robot_xy
        candidates = approach_candidates(
            robot_x,
            robot_y,
            object_x,
            object_y,
            float(self.get_parameter("standoff_distance").value),
            self.get_parameter("approach_angle_offsets_deg").value,
        )
        viable = []
        for index, (x, y, yaw) in enumerate(candidates):
            self._check_live_target()
            if handle.is_cancel_requested or time.monotonic() >= deadline:
                return None
            pose = self._pose(x, y, yaw)
            pose.header.stamp = self.get_clock().now().to_msg()
            length = await self._compute_path(pose, handle, deadline)
            if length is not None:
                viable.append((length, index, pose))
        if not viable:
            return None
        length, index, pose = min(viable, key=lambda item: (item[0], item[1]))
        self.get_logger().info(
            f"Pose de aproximação {index} selecionada; caminho={length:.2f} m"
        )
        return pose

    async def _request_vlm(self, query, handle, deadline):
        if time.monotonic() < self.vlm_retry_after:
            return False
        if not self.vlm_client.service_is_ready():
            self.vlm_retry_after = time.monotonic() + float(
                self.get_parameter("vlm_retry_interval_sec").value
            )
            self.get_logger().warning("Fallback VLM indisponível; mantendo busca local")
            return False
        request = DetectObject.Request()
        request.query = query
        future = self.vlm_client.call_async(request)
        try:
            while not future.done() and time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    future.cancel()
                    return False
                time.sleep(0.05)
            if not future.done():
                future.cancel()
                return False
            response = future.result()
            if not response.found:
                self.get_logger().warning(response.message)
                self.vlm_retry_after = time.monotonic() + float(
                    self.get_parameter("vlm_retry_interval_sec").value
                )
            return response.found
        except Exception as error:
            self.get_logger().warning(f"Fallback VLM falhou sem interromper a busca: {error}")
            self.vlm_retry_after = time.monotonic() + float(
                self.get_parameter("vlm_retry_interval_sec").value
            )
            return False

    async def _navigate(self, pose, handle, deadline):
        self._check_live_target()
        if handle.is_cancel_requested or time.monotonic() >= deadline:
            return False
        pose.header.stamp = self.get_clock().now().to_msg()
        request = NavigateToPose.Goal()
        request.pose = pose
        goal_future = self.nav_client.send_goal_async(request)
        while not goal_future.done() and time.monotonic() < deadline:
            if handle.is_cancel_requested:
                self._cancel_late_goal(goal_future)
                return False
            time.sleep(0.05)
        if not goal_future.done():
            self._cancel_late_goal(goal_future)
            return False
        goal = goal_future.result()
        if not goal.accepted:
            return False
        self.active_nav_goal = goal
        result_future = goal.get_result_async()
        try:
            while not result_future.done() and time.monotonic() < deadline:
                await self._guard_live_goal(goal, result_future)
                if handle.is_cancel_requested:
                    await self._cancel_subgoal(goal)
                    return False
                time.sleep(0.1)
            if not result_future.done():
                await self._cancel_subgoal(goal)
                return False
            wrapped = result_future.result()
            self._check_live_target()
            return wrapped.status == GoalStatus.STATUS_SUCCEEDED
        finally:
            if self.active_nav_goal is goal:
                self.active_nav_goal = None

    async def _navigate_until_seen(self, pose, label, handle, deadline):
        """Navigate to a waypoint and return only a fresh confirmed observation."""
        if handle.is_cancel_requested or time.monotonic() >= deadline:
            return False, None
        pose.header.stamp = self.get_clock().now().to_msg()
        request = NavigateToPose.Goal()
        request.pose = pose
        goal_future = self.nav_client.send_goal_async(request)
        while not goal_future.done() and time.monotonic() < deadline:
            if handle.is_cancel_requested:
                self._cancel_late_goal(goal_future)
                return False, None
            time.sleep(0.05)
        if not goal_future.done():
            self._cancel_late_goal(goal_future)
            return False, None
        goal = goal_future.result()
        if not goal.accepted:
            return False, None
        self.active_nav_goal = goal
        result_future = goal.get_result_async()
        count = 0
        last_sequence = None
        required = int(self.get_parameter("confirmation_count").value)
        try:
            while not result_future.done() and time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    await self._cancel_subgoal(goal)
                    return False, None
                match = self._matching_observation(label)
                if match is not None and match[0] != last_sequence:
                    last_sequence, observation = match
                    count += 1
                    if count >= required:
                        # Continue to the waypoint so SLAM explores the route.
                        # A distant observation may not have a plannable
                        # approach until that surrounding map exists.
                        count = required
                time.sleep(0.1)
            if not result_future.done():
                await self._cancel_subgoal(goal)
                return False, None
            wrapped = result_future.result()
            # A high-confidence sighting made earlier on the route can be
            # stale, including its coordinates while SLAM refines the map.
            fresh = self._matching_observation(label) if count >= required else None
            stable = fresh[1] if fresh is not None else None
            return wrapped.status == GoalStatus.STATUS_SUCCEEDED, stable
        finally:
            if self.active_nav_goal is goal:
                self.active_nav_goal = None

    async def _spin_segment(self, radians, handle, deadline):
        if handle.is_cancel_requested or time.monotonic() >= deadline:
            return False
        request = Spin.Goal()
        request.target_yaw = float(radians)
        request.time_allowance.sec = max(1, int(deadline - time.monotonic()))
        goal_future = self.spin_client.send_goal_async(request)
        while not goal_future.done() and time.monotonic() < deadline:
            if handle.is_cancel_requested:
                self._cancel_late_goal(goal_future)
                return False
            time.sleep(0.05)
        if not goal_future.done():
            self._cancel_late_goal(goal_future)
            return False
        goal = goal_future.result()
        if not goal.accepted:
            return False
        self.active_nav_goal = goal
        result_future = goal.get_result_async()
        try:
            while not result_future.done() and time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    await self._cancel_subgoal(goal)
                    return False
                time.sleep(0.1)
            if not result_future.done():
                await self._cancel_subgoal(goal)
                return False
            return result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        finally:
            if self.active_nav_goal is goal:
                self.active_nav_goal = None

    async def _spin_until_seen(self, radians, label, handle, deadline, expected=None):
        """Rotate while sampling perception, stopping as soon as a label is stable."""
        self._check_live_target()
        if handle.is_cancel_requested or time.monotonic() >= deadline:
            return False, None
        request = Spin.Goal()
        request.target_yaw = float(radians)
        request.time_allowance.sec = max(1, int(deadline - time.monotonic()))
        goal_future = self.spin_client.send_goal_async(request)
        while not goal_future.done() and time.monotonic() < deadline:
            if handle.is_cancel_requested:
                self._cancel_late_goal(goal_future)
                return False, None
            time.sleep(0.05)
        if not goal_future.done():
            self._cancel_late_goal(goal_future)
            return False, None
        goal = goal_future.result()
        if not goal.accepted:
            return False, None
        self.active_nav_goal = goal
        result_future = goal.get_result_async()
        count = 0
        last_sequence = None
        best = None
        required = int(self.get_parameter("confirmation_count").value)
        try:
            while not result_future.done() and time.monotonic() < deadline:
                await self._guard_live_goal(goal, result_future)
                if handle.is_cancel_requested:
                    await self._cancel_subgoal(goal)
                    return False, None
                match = self._matching_observation(label, expected)
                if match is not None and match[0] != last_sequence:
                    last_sequence, observation = match
                    count += 1
                    if best is None or observation[0] > best[0]:
                        best = observation
                    if count >= required:
                        await self._cancel_subgoal(goal)
                        return False, best
                time.sleep(0.1)
            if not result_future.done():
                await self._cancel_subgoal(goal)
                return False, None
            return result_future.result().status == GoalStatus.STATUS_SUCCEEDED, None
        finally:
            if self.active_nav_goal is goal:
                self.active_nav_goal = None

    async def _confirm(self, label, handle, deadline, expected=None):
        count, best = 0, None
        last_sequence = None
        stop = min(deadline, time.monotonic() + float(self.get_parameter("confirmation_timeout_sec").value))
        while time.monotonic() < stop:
            self._check_live_target()
            if handle.is_cancel_requested:
                return None
            match = self._matching_observation(label, expected)
            if match is not None and match[0] != last_sequence:
                last_sequence, observation = match
                count += 1
                best = observation if best is None or observation[0] > best[0] else best
                if count >= int(self.get_parameter("confirmation_count").value):
                    return best
            time.sleep(0.2)
        return None

    async def _reconfirm_with_scan(self, label, handle, deadline, expected=None):
        """Confirm in place, then actively reacquire the object if necessary."""
        observation = await self._confirm(label, handle, deadline, expected)
        if observation or handle.is_cancel_requested:
            return observation
        segments = max(1, int(self.get_parameter("spin_segments").value))
        for _ in range(segments):
            if time.monotonic() >= deadline or handle.is_cancel_requested:
                return None
            _, observation = await self._spin_until_seen(
                2.0 * math.pi / segments, label, handle, deadline, expected
            )
            if observation:
                return observation
            observation = await self._confirm(
                label, handle, min(deadline, time.monotonic() + 2.0), expected
            )
            if observation:
                return observation
        return None

    def _result(self, success, code, message, observation=None):
        result = GotoObject.Result()
        result.success, result.error_code, result.message = success, code, message
        if observation:
            result.object_pose.header.frame_id = "map"
            result.object_pose.header.stamp = self.get_clock().now().to_msg()
            result.object_pose.pose.position = observation[1]
            result.object_pose.pose.orientation.w = 1.0
        if self._mission_started is not None:
            result.elapsed_sec = float(time.monotonic() - self._mission_started)
            result.distance_traveled_m = float(
                max(0.0, self._odometry_distance_total - self._mission_distance_start)
            )
            result.points_visited = int(self._mission_points_visited)
            result.time_to_first_detection_sec = float(
                self._mission_first_detection_sec
            )
        return result

    async def _cancelled_result(self, handle, message="Busca cancelada"):
        await self._cancel_subgoal()
        handle.canceled()
        return self._result(False, GotoObject.Result.CANCELLED, message)

    async def _execute(self, handle):
        started = time.monotonic()
        self._mission_started = started
        self._mission_distance_start = self._odometry_distance_total
        self._mission_points_visited = 0
        self._mission_first_detection_sec = -1.0
        timeout = handle.request.timeout_sec or float(self.get_parameter("default_timeout_sec").value)
        deadline = started + max(1.0, timeout)
        environment = handle.request.environment_id.strip() or "default"
        original = handle.request.query.strip()
        label = normalize_query(original, self.aliases)
        self._search_had_navigation_failure = False
        try:
            if not label:
                handle.abort()
                return self._result(
                    False, GotoObject.Result.INVALID_QUERY,
                    "Consulta vazia: informe o nome ou a descrição de um objeto",
                )
            self._feedback(handle, GotoObject.Feedback.VALIDATING, f"Meta validada: {original} -> {label}", started)
            if not await self._wait_servers(handle, min(deadline, started + 15.0)):
                if handle.is_cancel_requested:
                    return await self._cancelled_result(handle)
                handle.abort()
                return self._result(False, GotoObject.Result.NAVIGATION_FAILED, "Actions do Nav2 indisponíveis")
            perception_deadline = min(
                deadline,
                time.monotonic()
                + float(self.get_parameter("sensor_startup_timeout_sec").value),
            )
            if not await self._wait_perception(handle, perception_deadline):
                if handle.is_cancel_requested:
                    return await self._cancelled_result(handle)
                handle.abort()
                return self._result(
                    False,
                    GotoObject.Result.SENSORS_UNAVAILABLE,
                    "Percepção indisponível: verifique câmera, CameraInfo, scan e TF",
                )
            robot_xy = self._robot_xy()
            if robot_xy is None:
                handle.abort()
                return self._result(
                    False, GotoObject.Result.SENSORS_UNAVAILABLE,
                    "Pose map -> base_footprint indisponível",
                )
            self._feedback(handle, GotoObject.Feedback.CHECKING_MAP, "Consultando mapa semântico", started)
            known = self.store.select(
                environment, label, self.get_parameter("instance_policy").value,
                robot_xy, self.get_parameter("map_checksum").value,
                ros_now_ns=self.get_clock().now().nanoseconds,
            )
            if known:
                visit = (self._visit_live_memory if known.get('external_id')
                         and self.get_parameter('live_external_targets').value else self._visit_memory)
                result = await visit(handle, environment, label, known, started, deadline)
                if result is not None:
                    return result
            use_vlm = requires_vlm(original, label)
            if use_vlm:
                await self._request_vlm(original, handle, deadline)
                if handle.is_cancel_requested:
                    return await self._cancelled_result(handle)
            observation = await self._confirm(
                label, handle, min(deadline, time.monotonic() + 1.0)
            )
            if observation:
                outcome = await self._approach_and_finish(
                    handle, environment, label, observation, started, deadline,
                    retry_on_navigation_failure=True,
                )
                if outcome is not None:
                    return outcome
            route, route_source = self._search_route(environment)
            self._feedback(
                handle, GotoObject.Feedback.SEARCHING,
                f"Iniciando busca por {route_source} ({len(route)} pontos)",
                started,
            )
            segments = max(1, int(self.get_parameter("spin_segments").value))
            for index, waypoint in enumerate(route):
                if handle.is_cancel_requested or time.monotonic() >= deadline:
                    break
                self._feedback(
                    handle, GotoObject.Feedback.SEARCHING,
                    f"Navegando ao ponto {index + 1}/{len(route)} de {route_source}",
                    started, index,
                )
                reached_waypoint, observation = await self._navigate_until_seen(
                    self._pose(
                        waypoint["x"], waypoint["y"], waypoint.get("yaw", 0.0)
                    ),
                    label, handle, deadline,
                )
                if reached_waypoint:
                    self._mission_points_visited += 1
                if handle.is_cancel_requested:
                    return await self._cancelled_result(handle)
                if observation:
                    outcome = await self._approach_and_finish(
                        handle, environment, label, observation, started, deadline,
                        retry_on_navigation_failure=True,
                    )
                    if outcome is not None:
                        return outcome
                observation = await self._confirm(
                    label, handle, min(deadline, time.monotonic() + 1.0)
                )
                if observation:
                    outcome = await self._approach_and_finish(
                        handle, environment, label, observation, started, deadline,
                        retry_on_navigation_failure=True,
                    )
                    if outcome is not None:
                        return outcome
                for _ in range(segments):
                    if use_vlm:
                        await self._request_vlm(original, handle, deadline)
                        if handle.is_cancel_requested:
                            return await self._cancelled_result(handle)
                    _, observation = await self._spin_until_seen(
                        2.0 * math.pi / segments, label, handle, deadline
                    )
                    if observation:
                        outcome = await self._approach_and_finish(
                            handle, environment, label, observation, started,
                            deadline, retry_on_navigation_failure=True,
                        )
                        if outcome is not None:
                            return outcome
            if handle.is_cancel_requested:
                return await self._cancelled_result(handle)
            handle.abort()
            if self._search_had_navigation_failure:
                return self._result(
                    False, GotoObject.Result.NAVIGATION_FAILED,
                    "Objeto detectado, mas nenhuma aproximação segura foi alcançável",
                )
            return self._result(False, GotoObject.Result.NOT_FOUND, "Objeto não encontrado antes do limite da busca")
        except Exception as error:  # action boundary: always return an intelligible result
            self.get_logger().error(f"Semantic navigation failed: {error}")
            if handle.is_cancel_requested:
                return await self._cancelled_result(handle)
            handle.abort()
            return self._result(False, GotoObject.Result.INTERNAL_ERROR, str(error))
        finally:
            with self._goal_state_lock:
                self._goal_active = False

    async def _visit_memory(self, handle, environment, label, known, started, deadline):
        self._check_live_target()
        self._feedback(handle, GotoObject.Feedback.NAVIGATING_TO_MEMORY, "Indo até coordenada salva", started)
        current_xy = self._robot_xy()
        already_close = current_xy is not None and within_standoff(
            current_xy[0], current_xy[1],
            known["pose"]["x"], known["pose"]["y"],
            float(self.get_parameter("standoff_distance").value),
            float(self.get_parameter("standoff_tolerance").value),
        )
        pose = None if already_close else await self._select_approach_pose(
            known["pose"]["x"], known["pose"]["y"], handle, deadline
        )
        if handle.is_cancel_requested:
            return await self._cancelled_result(handle)
        reached = already_close or (
            pose is not None and await self._navigate(pose, handle, deadline)
        )
        if handle.is_cancel_requested:
            return await self._cancelled_result(handle)
        if reached:
            self._feedback(handle, GotoObject.Feedback.RECONFIRMING, "Reconfirmando objeto visualmente", started)
            observation = await self._reconfirm_with_scan(
                label, handle, deadline, known
            )
            if handle.is_cancel_requested:
                return await self._cancelled_result(handle)
            self._check_live_target()
            if observation:
                return await self._finish_or_refine(
                    handle, environment, label, observation, started, deadline)
            self.store.mark_stale(environment, label, known["id"])
        else:
            if self._live_target is not None:
                handle.abort()
                return self._result(False, GotoObject.Result.NAVIGATION_FAILED,
                                    'Não foi possível alcançar a aproximação do objeto selecionado')
            self.get_logger().warning(
                "Coordenada memorizada não foi alcançada; iniciando busca sem invalidá-la"
            )
        return None

    async def _visit_live_memory(self, handle, environment, label, known, started, deadline):
        self._live_environment, self._live_label = environment, label
        distance = float(self.get_parameter('target_replan_distance_m').value)
        maximum = int(self.get_parameter('target_max_replans').value)
        try:
            for attempt in range(maximum + 1):
                self._live_target = LiveTarget(known, distance)
                try:
                    return await self._visit_memory(
                        handle, environment, label, known, started, deadline)
                except TargetMoved as error:
                    if handle.is_cancel_requested:
                        return await self._cancelled_result(handle)
                    if attempt == maximum or time.monotonic() >= deadline:
                        raise TargetLost('Objeto continuou mudando de posição; limite de aproximação atingido')
                    known = error.instance
                    self._feedback(handle, GotoObject.Feedback.NAVIGATING_TO_MEMORY,
                                   str(error), started)
        except TargetLost as error:
            await self._cancel_subgoal()
            if handle.is_cancel_requested:
                return await self._cancelled_result(handle)
            handle.abort()
            return self._result(False, GotoObject.Result.NOT_FOUND, str(error))
        finally:
            self._live_target = None

    async def _approach_and_finish(
            self, handle, environment, label, observation, started, deadline,
            retry_on_navigation_failure=False, refinements_remaining=1):
        self._feedback(handle, GotoObject.Feedback.OBJECT_LOCALIZED, "Objeto localizado pela câmera e lidar", started, confidence=observation[0])
        point = observation[1]
        self._feedback(handle, GotoObject.Feedback.APPROACHING, "Aproximando com distância segura", started, confidence=observation[0])
        current_xy = self._robot_xy()
        already_close = current_xy is not None and within_standoff(
            current_xy[0], current_xy[1], point.x, point.y,
            float(self.get_parameter("standoff_distance").value),
            float(self.get_parameter("standoff_tolerance").value),
        )
        pose = None if already_close else await self._select_approach_pose(
            point.x, point.y, handle, deadline
        )
        if handle.is_cancel_requested:
            return await self._cancelled_result(handle)
        if pose is None and not already_close:
            if retry_on_navigation_failure:
                self._search_had_navigation_failure = True
                self.get_logger().warning(
                    "Aproximação ainda sem caminho; continuando a exploração"
                )
                return None
            handle.abort()
            return self._result(
                False, GotoObject.Result.NAVIGATION_FAILED,
                "Nenhuma pose segura de aproximação possui caminho válido",
                observation,
            )
        if not already_close and not await self._navigate(pose, handle, deadline):
            if handle.is_cancel_requested:
                return await self._cancelled_result(handle)
            if retry_on_navigation_failure:
                self._search_had_navigation_failure = True
                self.get_logger().warning(
                    "Aproximação falhou; continuando a busca por outra observação"
                )
                return None
            handle.abort()
            return self._result(False, GotoObject.Result.NAVIGATION_FAILED, "Nav2 não alcançou a pose de aproximação", observation)
        confirmed = await self._reconfirm_with_scan(label, handle, deadline)
        if handle.is_cancel_requested:
            return await self._cancelled_result(handle)
        if not confirmed:
            if retry_on_navigation_failure:
                self.get_logger().warning(
                    "Objeto perdido na aproximação; retomando busca"
                )
                return None
            handle.abort()
            return self._result(False, GotoObject.Result.NOT_FOUND, "Objeto perdido durante a aproximação")
        return await self._finish_or_refine(
            handle, environment, label, confirmed, started, deadline,
            refinements_remaining=refinements_remaining)

    async def _finish_or_refine(
            self, handle, environment, label, observation, started, deadline,
            refinements_remaining=1):
        """Correct one approach if reconfirmation refines the object position."""
        if handle.is_cancel_requested:
            return await self._cancelled_result(handle)
        point = observation[1]
        robot_xy = self._robot_xy()
        close = robot_xy is not None and within_standoff(
            *robot_xy, point.x, point.y,
            float(self.get_parameter("standoff_distance").value),
            float(self.get_parameter("standoff_tolerance").value))
        if not close and refinements_remaining > 0 and time.monotonic() < deadline:
            self.get_logger().info("Posição reconfirmada mudou; corrigindo aproximação uma vez")
            return await self._approach_and_finish(
                handle, environment, label, observation, started, deadline,
                refinements_remaining=refinements_remaining - 1)
        return await self._finish(handle, environment, label, observation, started)

    async def _finish(self, handle, environment, label, observation, started):
        if handle.is_cancel_requested:
            return await self._cancelled_result(handle)
        self._check_live_target()
        point = observation[1]
        robot_xy = self._robot_xy()
        if robot_xy is None or not within_standoff(
                *robot_xy, point.x, point.y,
                float(self.get_parameter("standoff_distance").value),
                float(self.get_parameter("standoff_tolerance").value)):
            handle.abort()
            return self._result(
                False, GotoObject.Result.NAVIGATION_FAILED,
                "Objeto confirmado, mas a distância final de aproximação é inválida",
                observation,
            )
        self._feedback(handle, GotoObject.Feedback.PERSISTING, "Atualizando mapa semântico", started, confidence=observation[0])
        self.store.upsert(
            environment, label, {"x": point.x, "y": point.y, "yaw": 0.0}, observation[0],
            float(self.get_parameter("merge_distance").value), self.get_parameter("map_checksum").value,
            observation_ros_stamp_ns=self.get_clock().now().nanoseconds,
            instance_id=self._live_target.instance['id'] if self._live_target else None,
        )
        handle.succeed()
        return self._result(True, GotoObject.Result.SUCCEEDED, "Objeto alcançado e confirmado", observation)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigationServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
