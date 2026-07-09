#!/usr/bin/env python3

import sys
import rclpy
from nav2_msgs.srv import LoadMap


def load_map_service(map_path):
    """Call the load_map service to load a map file."""
    rclpy.init()
    node = rclpy.create_node('map_loader')
    
    cli = node.create_client(LoadMap, 'map_server/load_map')
    
    # Wait for service to be available
    while not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().info('Service map_server/load_map not available, waiting...')
    
    req = LoadMap.Request()
    req.map_url = map_path
    
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info(f'Map loaded successfully: {map_path}')
    else:
        node.get_logger().error(f'Failed to load map: {map_path}')
    
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: load_map.py <map_path>")
        sys.exit(1)
    
    map_path = sys.argv[1]
    load_map_service(map_path)
