#!/usr/bin/env python3
"""
Launch file for Orbbec Camera Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'rgb_device',
            default_value='0',
            description='RGB camera device number'
        ),
        DeclareLaunchArgument(
            'depth_device',
            default_value='1',
            description='Depth camera device number'
        ),
        DeclareLaunchArgument(
            'frame_rate',
            default_value='15.0',
            description='Camera frame rate'
        ),
        
        # Camera node
        Node(
            package='my_custom_nodes',
            executable='orbbec_camera_node.py',
            name='orbbec_camera_node',
            output='screen',
            parameters=[{
                'rgb_device': LaunchConfiguration('rgb_device'),
                'depth_device': LaunchConfiguration('depth_device'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'rgb_width': 640,
                'rgb_height': 480,
                'depth_width': 640,
                'depth_height': 480,
                'camera_frame': 'camera_link',
                'rgb_optical_frame': 'camera_rgb_optical_frame',
                'depth_optical_frame': 'camera_depth_optical_frame',
            }]
        ),
        
        # Static transform: base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link']
        ),
        
        # Static transform: camera_link -> camera_rgb_optical_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_rgb_optical_tf',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_rgb_optical_frame']
        ),
        
        # Static transform: camera_link -> camera_depth_optical_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_depth_optical_tf',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_depth_optical_frame']
        ),
    ])
