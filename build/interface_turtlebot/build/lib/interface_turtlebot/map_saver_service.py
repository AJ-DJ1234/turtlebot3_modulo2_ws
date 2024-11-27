#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
import os
import subprocess

class MapSaverService(Node):
    def __init__(self):
        super().__init__('map_saver_service')
        self.callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(Trigger, 'save_map', self.save_map_callback, callback_group=self.callback_group)

    def save_map_callback(self, request, response):
        self.get_logger().info('Received request to save map')
        save_command = 'ros2 run nav2_map_server map_saver_cli -f ~/Workspaces/turtlebot3_ws/map_slam/map'
        try:
            subprocess.run(save_command, shell=True, check=True)
            response.success = True
            response.message = 'Map saved successfully'
        except subprocess.CalledProcessError as e:
            response.success = False
            response.message = f'Failed to save map: {e}'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MapSaverService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
