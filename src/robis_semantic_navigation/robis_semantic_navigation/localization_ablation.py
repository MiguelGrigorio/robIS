#!/usr/bin/env python3
"""Run one localization-estimator sequence through persistent ROS clients."""

import argparse
import json
import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import Spin
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


class LocalizationAblation(Node):
    def __init__(self):
        super().__init__("localization_ablation")
        self.evaluator = AsyncParameterClient(self, "/localization_evaluator")
        self.localizer = AsyncParameterClient(self, "/object_localizer")
        self.detector_evaluator = AsyncParameterClient(self, "/detector_evaluator")
        self.detector_evaluator_required = False
        self.spin_action = ActionClient(self, Spin, "/spin")
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.current_yaw = None
        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)

    def _odom_callback(self, message):
        orientation = message.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )

    def wait_until_ready(self, timeout_sec=30.0, require_detector_evaluator=False,
                         rotation_backend="nav2"):
        if not self.evaluator.wait_for_services(timeout_sec=timeout_sec):
            raise RuntimeError("localization evaluator parameter service unavailable")
        if not self.localizer.wait_for_services(timeout_sec=timeout_sec):
            raise RuntimeError("object localizer parameter service unavailable")
        if rotation_backend == "nav2" and not self.spin_action.wait_for_server(
            timeout_sec=timeout_sec
        ):
            raise RuntimeError("spin action unavailable")
        self.detector_evaluator_required = bool(require_detector_evaluator)
        if require_detector_evaluator and not self.detector_evaluator.wait_for_services(
            timeout_sec=timeout_sec
        ):
            raise RuntimeError("detector evaluator parameter service unavailable")

    def set_parameters(self, client, parameters, timeout_sec=15.0):
        future = client.set_parameters(parameters)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            raise RuntimeError("parameter request timed out")
        failures = [
            result.reason for result in future.result().results
            if not result.successful
        ]
        if failures:
            raise RuntimeError("parameter rejected: " + "; ".join(failures))

    def configure_origin(self, x, y, yaw):
        self.set_parameters(self.evaluator, [
            Parameter("world_to_map_origin_x", Parameter.Type.DOUBLE, float(x)),
            Parameter("world_to_map_origin_y", Parameter.Type.DOUBLE, float(y)),
            Parameter("world_to_map_origin_yaw", Parameter.Type.DOUBLE, float(yaw)),
        ])

    def rotate_direct(self, spin_radians, time_allowance, angular_speed=0.45):
        deadline = time.monotonic() + 10.0
        while self.current_yaw is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.current_yaw is None:
            raise RuntimeError("odometry unavailable for direct rotation")
        previous = self.current_yaw
        accumulated = 0.0
        started = time.monotonic()
        command = Twist()
        command.angular.z = abs(float(angular_speed))
        try:
            while accumulated < abs(float(spin_radians)):
                if time.monotonic() - started > float(time_allowance):
                    raise RuntimeError("direct rotation timed out")
                self.cmd_vel.publish(command)
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.current_yaw is None:
                    continue
                delta = math.atan2(
                    math.sin(self.current_yaw - previous),
                    math.cos(self.current_yaw - previous),
                )
                accumulated += max(delta, 0.0)
                previous = self.current_yaw
        finally:
            stop = Twist()
            for _ in range(5):
                self.cmd_vel.publish(stop)
                rclpy.spin_once(self, timeout_sec=0.02)
        return {
            "rotation_rad": accumulated,
            "rotation_duration_sec": time.monotonic() - started,
        }

    def collect(self, estimator, trial_id, spin_radians, time_allowance,
                rotation_backend="nav2"):
        experiment_trial = f"{trial_id}_{estimator}"
        self.set_parameters(self.evaluator, [
            Parameter("recording_enabled", Parameter.Type.BOOL, False),
            Parameter("range_estimator", Parameter.Type.STRING, estimator),
            Parameter("trial_id", Parameter.Type.STRING, experiment_trial),
        ])
        self.set_parameters(self.localizer, [
            Parameter("range_estimator", Parameter.Type.STRING, estimator),
        ])
        if self.detector_evaluator_required:
            self.set_parameters(self.detector_evaluator, [
                Parameter("recording_enabled", Parameter.Type.BOOL, False),
                Parameter("trial_id", Parameter.Type.STRING, experiment_trial),
            ])
        self.set_parameters(self.evaluator, [
            Parameter("recording_enabled", Parameter.Type.BOOL, True),
        ])
        if self.detector_evaluator_required:
            self.set_parameters(self.detector_evaluator, [
                Parameter("recording_enabled", Parameter.Type.BOOL, True),
            ])
        if rotation_backend == "cmd_vel":
            rotation = self.rotate_direct(spin_radians, time_allowance)
        else:
            goal = Spin.Goal()
            goal.target_yaw = float(spin_radians)
            goal.time_allowance = Duration(seconds=float(time_allowance)).to_msg()
            goal_future = self.spin_action.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, goal_future, timeout_sec=15.0)
            if not goal_future.done() or goal_future.result() is None:
                raise RuntimeError(f"{estimator}: spin goal was not accepted")
            handle = goal_future.result()
            if not handle.accepted:
                raise RuntimeError(f"{estimator}: spin goal rejected")
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=float(time_allowance) + 30.0
            )
            if not result_future.done() or result_future.result() is None:
                raise RuntimeError(f"{estimator}: spin action timed out")
            status = result_future.result().status
            if status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f"{estimator}: spin finished with status {status}")
            rotation = {
                "rotation_rad": abs(float(spin_radians)),
                "rotation_duration_sec": None,
            }
        self.set_parameters(self.evaluator, [
            Parameter("recording_enabled", Parameter.Type.BOOL, False),
        ])
        if self.detector_evaluator_required:
            self.set_parameters(self.detector_evaluator, [
                Parameter("recording_enabled", Parameter.Type.BOOL, False),
            ])
        return {
            "estimator": estimator,
            "trial_id": trial_id,
            "status": "succeeded",
            "rotation_backend": rotation_backend,
            **rotation,
        }


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial-id", required=True)
    parser.add_argument("--estimators", nargs="+", required=True)
    parser.add_argument("--spin-radians", type=float, default=6.283185307)
    parser.add_argument("--time-allowance", type=float, default=90.0)
    parser.add_argument("--origin", type=float, nargs=3, metavar=("X", "Y", "YAW"))
    parser.add_argument("--require-detector-evaluator", action="store_true")
    parser.add_argument(
        "--rotation-backend", choices=("nav2", "cmd_vel"), default="nav2"
    )
    parsed = parser.parse_args(args=args)
    rclpy.init()
    node = LocalizationAblation()
    try:
        node.wait_until_ready(
            require_detector_evaluator=parsed.require_detector_evaluator,
            rotation_backend=parsed.rotation_backend,
        )
        if parsed.origin:
            node.configure_origin(*parsed.origin)
        for estimator in parsed.estimators:
            result = node.collect(
                estimator, parsed.trial_id, parsed.spin_radians,
                parsed.time_allowance,
                parsed.rotation_backend,
            )
            print("ABLATION_RESULT " + json.dumps(result, sort_keys=True))
    finally:
        try:
            node.set_parameters(node.evaluator, [
                Parameter("recording_enabled", Parameter.Type.BOOL, False),
            ])
        except RuntimeError:
            pass
        if node.detector_evaluator_required:
            try:
                node.set_parameters(node.detector_evaluator, [
                    Parameter("recording_enabled", Parameter.Type.BOOL, False),
                ])
            except RuntimeError:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
