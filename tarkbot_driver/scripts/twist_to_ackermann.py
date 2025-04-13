# 在twist_to_ackermann.py节点中增加运动学计算
import math
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from math import atan2

class TwistConverter(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.convert, 10)
        self.pub = self.create_publisher(JointState, 'joint_commands', 10)
        
        self.wheelbase = 2.0  # 与URDF一致
        self.track_width = 1.5
        
    
    def convert(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        
        # 阿克曼转向几何计算
        if math.isclose(v, 0.0, rel_tol=1e-3):
            steering_angle = 0.0
        else:
            radius = v / omega if omega != 0 else float('inf')
            steering_angle = math.atan(self.wheelbase / radius)
        
        # 转向角度限幅（±30度）
        steering_angle = max(min(steering_angle, math.radians(30)), math.radians(-30))
        
        # 构造JointState消息
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = [
            'front_left_steering_joint', 
            'front_right_steering_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        cmd.position = [steering_angle, steering_angle, 0.0, 0.0]
        cmd.velocity = [0.0, 0.0, v, v]  # 后轮速度
        
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = TwistConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
