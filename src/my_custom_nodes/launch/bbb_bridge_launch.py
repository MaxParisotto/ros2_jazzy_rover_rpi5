"""
Launch file for BeagleBone Blue Bridge Node

This launches the BBB bridge node that communicates with the 
BeagleBone Blue motor controller via UART.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyAMA0',
        description='Serial port for BBB communication'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.2',
        description='Distance between left and right wheels (meters)'
    )
    
    wheel_track_arg = DeclareLaunchArgument(
        'wheel_track',
        default_value='0.15',
        description='Distance between front and rear wheels (meters)'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.05',
        description='Wheel radius (meters)'
    )
    
    encoder_ticks_arg = DeclareLaunchArgument(
        'encoder_ticks_per_rev',
        default_value='1440',
        description='Encoder ticks per wheel revolution'
    )
    
    # BBB Bridge Node
    bbb_bridge_node = Node(
        package='my_custom_nodes',
        executable='bbb_bridge_node.py',
        name='bbb_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_track': LaunchConfiguration('wheel_track'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'encoder_ticks_per_rev': LaunchConfiguration('encoder_ticks_per_rev'),
            'max_motor_speed': 1.0,
            'publish_tf': True,
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        wheel_base_arg,
        wheel_track_arg,
        wheel_radius_arg,
        encoder_ticks_arg,
        bbb_bridge_node,
    ])
