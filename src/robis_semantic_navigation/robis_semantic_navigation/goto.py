#!/usr/bin/env python3
"""Small command-line client for the ``goto_object`` action."""

import argparse
import json
from pathlib import Path
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robis_interfaces.action import GotoObject


class GotoClient(Node):
    def __init__(self, result_json=None):
        super().__init__("goto_object_cli")
        self.client = ActionClient(self, GotoObject, "goto_object")
        self.exit_code = 1
        self.goal_handle = None
        self.result_json = Path(result_json) if result_json else None

    def send(self, query, environment, timeout):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Servidor /goto_object indisponível")
            rclpy.shutdown()
            return
        goal = GotoObject.Goal()
        goal.query, goal.environment_id, goal.timeout_sec = query, environment, timeout
        future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._accepted)

    def _feedback(self, message):
        feedback = message.feedback
        self.get_logger().info(
            f"[{feedback.elapsed_sec:6.1f}s] {feedback.description} "
            f"(confiança={feedback.best_confidence:.2f})"
        )

    def _accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Meta rejeitada")
            rclpy.shutdown()
            return
        self.goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(self._done)

    def _done(self, future):
        result = future.result().result
        log = self.get_logger().info if result.success else self.get_logger().error
        log(result.message)
        metrics = {
            "success": bool(result.success),
            "error_code": int(result.error_code),
            "message": result.message,
            "elapsed_sec": float(result.elapsed_sec),
            "distance_traveled_m": float(result.distance_traveled_m),
            "points_visited": int(result.points_visited),
            "time_to_first_detection_sec": float(
                result.time_to_first_detection_sec
            ),
            "object_pose": {
                "x": float(result.object_pose.pose.position.x),
                "y": float(result.object_pose.pose.position.y),
                "z": float(result.object_pose.pose.position.z),
            },
        }
        self.get_logger().info(f"MISSION_METRICS {json.dumps(metrics, sort_keys=True)}")
        if self.result_json is not None:
            self.result_json.parent.mkdir(parents=True, exist_ok=True)
            self.result_json.write_text(
                json.dumps(metrics, indent=2, sort_keys=True), encoding="utf-8"
            )
        self.exit_code = 0 if result.success else int(result.error_code or 1)
        rclpy.shutdown()

    def cancel(self):
        if self.goal_handle is None:
            return None
        self.get_logger().warning("Cancelando meta ativa antes de encerrar")
        self.exit_code = 130
        return self.goal_handle.cancel_goal_async()


def main(args=None):
    parser = argparse.ArgumentParser(description="Navegue até um objeto por nome")
    parser.add_argument("object", help="classe ou descrição do objeto")
    parser.add_argument("--environment", default="default")
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument(
        "--result-json", default="", help="grave o resultado e métricas em JSON"
    )
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = GotoClient(parsed.result_json)
    node.send(parsed.object, parsed.environment, parsed.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cancel_future = node.cancel()
        if cancel_future is not None and rclpy.ok():
            rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=3.0)
    code = node.exit_code
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if args is None:
        sys.exit(code)
