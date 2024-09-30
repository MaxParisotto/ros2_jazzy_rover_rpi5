from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=['/home/max/ros2_ws/config/planner_params.yaml'],
            arguments=['--ros-args', '--log-level', 'debug']
        )
    ])
