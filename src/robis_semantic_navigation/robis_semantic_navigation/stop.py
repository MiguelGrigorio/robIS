"""Cancel semantic and coordinate navigation through their ROS action services."""
import json

from action_msgs.srv import CancelGoal
import rclpy
from rclpy.node import Node


def cancel_actions(node):
    results = {}
    for action in ['goto_object', 'navigate_to_pose', 'spin']:
        client = node.create_client(CancelGoal, f'/{action}/_action/cancel_goal')
        try:
            if not client.wait_for_service(timeout_sec=2.):
                results[action] = {'available': False}
                continue
            future = client.call_async(CancelGoal.Request())  # Zero UUID/time cancels all goals.
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            response = future.result() if future.done() else None
            results[action] = {'available': True, 'response': response.return_code if response else None,
                               'goals_canceling': len(response.goals_canceling) if response else 0}
        finally:
            node.destroy_client(client)
    return results


def main(args=None):
    rclpy.init(args=args)
    node = Node('robis_stop_navigation')
    try:
        result = cancel_actions(node)
        print(json.dumps(result), flush=True)
        if not any(value.get('available') for value in result.values()):
            raise RuntimeError('Nenhum servidor de navegação disponível')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
