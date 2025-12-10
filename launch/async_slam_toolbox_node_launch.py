from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Map directory
    map_dir = '/home/max/ros2_ws/map'
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value='/home/max/ros2_ws/config/slam_toolbox_params.yaml',
            description='SLAM parameters file'
        ),
        
        # SLAM Toolbox node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),
    ])
