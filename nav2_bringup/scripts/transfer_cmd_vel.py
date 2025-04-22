#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistConverter(Node):
    def __init__(self):
        super().__init__('twist_converter')
        
        # 创建订阅者订阅原始cmd_vel话题
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.callback,
            1  # QoS队列大小
        )
        
        # 创建发布器用于发布转换后的消息
        self.publisher = self.create_publisher(
            TwistStamped,
            '/ackermann_steering_controller/reference',
            1  # QoS队列大小
        )
        
        self.get_logger().info('Twist转换节点已启动')

    def callback(self, msg):
        # 创建TwistStamped消息
        stamped_msg = TwistStamped()
        
        # 添加时间戳和坐标系
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 复制原始速度数据
        stamped_msg.twist = msg
        
        # 发布转换后的消息
        self.publisher.publish(stamped_msg)
        self.get_logger().debug('已转换并发布速度命令')

def main(args=None):
    rclpy.init(args=args)
    converter = TwistConverter()
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()