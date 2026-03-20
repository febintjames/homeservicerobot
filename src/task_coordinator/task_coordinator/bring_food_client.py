#!/usr/bin/env python3
"""
Simple client to trigger the /bring_food service.

Usage:
    ros2 run task_coordinator bring_food

Author: Febin TJ
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class BringFoodClient(Node):
    """Client node that calls the /bring_food service to start the task."""

    def __init__(self):
        super().__init__('bring_food_client')
        self.client = self.create_client(Trigger, 'bring_food')

        self.get_logger().info('Waiting for /bring_food service...')
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting...')

        self.send_request()

    def send_request(self):
        """Send trigger request to start food delivery."""
        request = Trigger.Request()
        self.get_logger().info('🚀 Sending BRING FOOD command...')

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ {response.message}')
            else:
                self.get_logger().error(f'❌ {response.message}')
        else:
            self.get_logger().error('Service call failed!')


def main(args=None):
    rclpy.init(args=args)
    node = BringFoodClient()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
