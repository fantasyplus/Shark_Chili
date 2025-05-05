#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port', default='/dev/ttyACM0')
    baud = LaunchConfiguration('baud', default='230400')
    robot_type = LaunchConfiguration('robot_type', default='r20_akm')
    pub_odom_tf = LaunchConfiguration('pub_odom_tf', default='true')
    base_footprint_frame = LaunchConfiguration('base_footprint_frame', default='base_footprint')
    base_link_frame = LaunchConfiguration(
        'base_link_frame', default='base_link')
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value=port,
            description='Specifying usb port to connected robot'),
        DeclareLaunchArgument(
            'baud',
            default_value=baud,
            description='Specifying usb port baudrate to connected robot'),
        DeclareLaunchArgument(
            'robot_type',
            default_value=robot_type,
            description='Specifying robot type'),
        DeclareLaunchArgument(
            'pub_odom_tf',
            default_value=pub_odom_tf,
            description='Specifying whether or not to publish odom tf'),
        DeclareLaunchArgument(
            'base_footprint_frame',
            default_value=base_footprint_frame,
            description='Specifying base frame'),
        DeclareLaunchArgument(
            'base_link_frame',
            default_value=base_link_frame,
            description='Specifying base link frame'),
        DeclareLaunchArgument(
            'imu_frame',
            default_value=imu_frame,
            description='Specifying imu frame'),

        Node(
            package='tarkbot_driver',
            executable='tarkbot_driver_node',
            name='tarkbot_driver_node',
            parameters=[{'port': port,
                         'baud': baud,
                         'robot_type': robot_type,
                         'pub_odom_tf': pub_odom_tf,
                         'base_footprint_frame': base_footprint_frame,
                         'imu_frame': imu_frame,
                         'base_link_frame': base_link_frame}],
            output='screen'),
    ])
