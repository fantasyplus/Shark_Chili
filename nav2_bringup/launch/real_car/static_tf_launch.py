#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    base_footprint_frame = LaunchConfiguration('base_footprint_frame', default='base_footprint')
    base_link_frame = LaunchConfiguration('base_link_frame', default='base_link')
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')
    laser_frame = LaunchConfiguration('laser_frame', default='base_scan')

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_base_link',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0.05',
                '--frame-id', base_footprint_frame,
                '--child-frame-id', base_link_frame
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0.08',
                '--yaw', '3.14159',
                '--frame-id', base_link_frame,
                '--child-frame-id', laser_frame
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0.01',
                '--frame-id', base_link_frame,
                '--child-frame-id', imu_frame
            ]
        ),
    ])
