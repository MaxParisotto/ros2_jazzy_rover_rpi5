from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, LogInfo
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Static transform publisher (map -> odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # Static transform publisher (odom -> base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),

        # Static transform publisher (base_link -> laser_frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
            output='screen'
        ),
        # Static transform publisher (base_link -> imu_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),

        # SLAM toolbox node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='localization',
            output='screen',
            parameters=['/home/max/ros2_ws/config/slam_toolbox_params.yaml']
        ),

        # Planner server for path planning
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=['/home/max/ros2_ws/config/planner_params.yaml']
        ),

        # Controller server with debug logging
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=['/home/max/ros2_ws/config/controller_params.yaml']
        ),

        # Waypoint follower for waypoint navigation
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=['/home/max/ros2_ws/config/waypoint_params.yaml']
        ),

        # IMU Subscriber from your custom package
        Node(
            package='my_custom_nodes',
            executable='imu_subscriber',
            name='imu_subscriber',
            output='screen',
            remappings=[('/imu/data_raw', '/imu/data_raw')]
        ),

        # Motor Encoders Subscriber
        Node(
            package='my_custom_nodes',
            executable='encoder_subscriber',
            name='encoder_subscriber',
            output='screen',
            remappings=[('/motor_encoders', '/motor_encoders')]
        ),

        # Motor Commands Publisher
        Node(
            package='my_custom_nodes',
            executable='motor_command_publisher',
            name='motor_command_publisher',
            output='screen',
            remappings=[('/motor_commands', '/motor_commands')]
        ),

        # Map server for loading and serving the map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/max/ros2_ws/my_map.yaml',
                'use_sim_time': False
            }]
        ),

        # Timer action to save the map automatically every 30 seconds
        TimerAction(
            period=5.0,  # Time period in seconds
            actions=[
                LogInfo(msg="Saving the map using slam_toolbox service..."),
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/slam_toolbox/save_map',
                        'slam_toolbox/srv/SaveMap',
                        '{"name": "/home/max/ros2_ws/my_map"}'
                    ],
                    output='screen'
                )
            ]
        )
    ])