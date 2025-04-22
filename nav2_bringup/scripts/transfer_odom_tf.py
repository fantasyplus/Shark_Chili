#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFBridge(Node):
    def __init__(self):
        super().__init__('tf_bridge')
        self.subscription = self.create_subscription(
            TFMessage,
            '/ackermann_steering_controller/tf_odometry',
            self.callback,
            10)
        self.publisher = self.create_publisher(TFMessage, '/tf', 10)

    def callback(self, msg):
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TFBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()