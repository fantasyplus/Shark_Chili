#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

class AckermannControllerBridge(Node):
    def __init__(self):
        super().__init__('tf_bridge')
        self.subscription = self.create_subscription(
            TFMessage,
            '/ackermann_steering_controller/tf_odometry',
            self.callback_tf,
            10)
        self.subscription = self.create_subscription(
            Odometry,
            '/ackermann_steering_controller/odometry',
            self.callback_odom,
            10)
        self.publisher_tf = self.create_publisher(TFMessage, '/tf', 10)
        self.publisher_odom = self.create_publisher(Odometry, '/odom', 10)

    def callback_tf(self, msg):
        self.publisher_tf.publish(msg)
    
    def callback_odom(self, msg):
        self.publisher_odom.publish(msg)

def main():
    rclpy.init()
    node = AckermannControllerBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()