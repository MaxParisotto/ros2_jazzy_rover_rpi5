from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=['/home/max/ros2_ws/config/amcl_params.yaml'],
            arguments=['--ros-args', '--log-level', 'debug']
        )
    ])
