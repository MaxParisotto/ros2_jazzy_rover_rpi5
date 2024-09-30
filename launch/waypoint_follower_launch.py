from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=['/home/max/ros2_ws/config/waypoint_follower_params.yaml'],
            arguments=['--ros-args', '--log-level', 'debug']
        )
    ])
