#!/usr/bin/env python3
"""
Master Launch File for Rover System
Launches: BBB Bridge, LiDAR, Camera, SLAM, Foxglove Bridge
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Package paths
    my_custom_nodes_share = FindPackageShare('my_custom_nodes')
    ldlidar_share = FindPackageShare('ldlidar_stl_ros2')
    
    return LaunchDescription([
        # ========== ARGUMENTS ==========
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'enable_camera',
            default_value='true',
            description='Enable camera node'
        ),
        DeclareLaunchArgument(
            'enable_slam',
            default_value='true',
            description='Enable SLAM'
        ),
        DeclareLaunchArgument(
            'enable_foxglove',
            default_value='true',
            description='Enable Foxglove WebSocket bridge'
        ),
        
        # ========== BBB BRIDGE (Motors, IMU, Odom, Battery) ==========
        Node(
            package='my_custom_nodes',
            executable='bbb_bridge_node.py',
            name='bbb_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyAMA0',
                'baud_rate': 115200,
                'wheel_base': 0.2,
                'wheel_radius': 0.05,
            }]
        ),
        
        # ========== LIDAR ==========
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([ldlidar_share, 'launch', 'ld19.launch.py'])
            ])
        ),
        
        # ========== CAMERA ==========
        Node(
            package='my_custom_nodes',
            executable='orbbec_camera_node.py',
            name='orbbec_camera_node',
            output='screen',
            parameters=[{
                'rgb_device': 0,
                'depth_device': 1,
                'frame_rate': 15.0,
            }],
            # condition=LaunchConfigurationEquals('enable_camera', 'true')
        ),
        
        # Camera TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_rgb_optical_tf',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_rgb_optical_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_depth_optical_tf',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_depth_optical_frame']
        ),
        
        # ========== SLAM TOOLBOX (delayed to let sensors stabilize) ==========
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=['/home/max/ros2_ws/config/slam_toolbox_params.yaml'],
                ),
            ]
        ),
        
        # ========== AUTO SAVE MAP ==========
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='my_custom_nodes',
                    executable='save_map_node.py',
                    name='auto_save_map_node',
                    output='screen',
                    parameters=[{
                        'save_interval': 60.0,
                        'map_dir': '/home/max/ros2_ws/map',
                        'map_name': 'persistent_map',
                    }]
                ),
            ]
        ),
        
        # ========== FOXGLOVE BRIDGE ==========
        Node(
            package='my_custom_nodes',
            executable='foxglove_bridge_node.py',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 9090,
                'host': '0.0.0.0',
            }]
        ),
    ])
