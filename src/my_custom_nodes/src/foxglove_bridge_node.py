#!/usr/bin/env python3
"""
ROS2 to Foxglove WebSocket Bridge
Exposes ROS2 topics via WebSocket on port 9090 for Foxglove Studio
Uses the rosbridge protocol for compatibility
"""

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import LaserScan, Image, Imu, BatteryState, CameraInfo
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
import asyncio
import websockets
import json
import threading
import base64
import struct
from datetime import datetime


class FoxgloveBridge(Node):
    def __init__(self):
        super().__init__('foxglove_bridge')
        
        # Parameters
        self.declare_parameter('port', 9090)
        self.declare_parameter('host', '0.0.0.0')
        
        self.port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        
        # Connected clients
        self.ws_clients = set()
        self.lock = threading.Lock()
        
        # Topic data cache (most recent message)
        self.topic_data = {}
        
        # Subscribers
        self._setup_subscribers()
        
        # WebSocket server runs in separate thread
        self.ws_thread = threading.Thread(target=self._run_ws_server, daemon=True)
        self.ws_thread.start()
        
        self.get_logger().info(f'Foxglove Bridge started on ws://{self.host}:{self.port}')
    
    def _setup_subscribers(self):
        """Set up ROS2 subscribers for common topics"""
        # LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', 
            lambda msg: self._cache_message('/scan', 'sensor_msgs/LaserScan', msg), 10)
        
        # Odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            lambda msg: self._cache_message('/odom', 'nav_msgs/Odometry', msg), 10)
        
        # IMU
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw',
            lambda msg: self._cache_message('/imu/data_raw', 'sensor_msgs/Imu', msg), 10)
        
        # Battery
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state',
            lambda msg: self._cache_message('/battery_state', 'sensor_msgs/BatteryState', msg), 10)
        
        # Map
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map',
            lambda msg: self._cache_message('/map', 'nav_msgs/OccupancyGrid', msg), 10)
        
        # TF
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf',
            lambda msg: self._cache_message('/tf', 'tf2_msgs/TFMessage', msg), 10)
        
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static',
            lambda msg: self._cache_message('/tf_static', 'tf2_msgs/TFMessage', msg), 10)
        
        # Camera (if available)
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            lambda msg: self._cache_message('/camera/color/image_raw', 'sensor_msgs/Image', msg), 5)
    
    def _cache_message(self, topic, msg_type, msg):
        """Cache the latest message for a topic"""
        with self.lock:
            self.topic_data[topic] = {
                'type': msg_type,
                'msg': msg,
                'timestamp': self.get_clock().now().nanoseconds
            }
        
        # Broadcast to connected clients
        asyncio.run_coroutine_threadsafe(
            self._broadcast_message(topic, msg_type, msg),
            self.loop
        )
    
    def _msg_to_dict(self, msg):
        """Convert ROS2 message to dictionary"""
        result = {}
        
        for field in msg.get_fields_and_field_types().keys():
            value = getattr(msg, field)
            
            if hasattr(value, 'get_fields_and_field_types'):
                # Nested message
                result[field] = self._msg_to_dict(value)
            elif isinstance(value, (list, tuple)):
                if len(value) > 0 and hasattr(value[0], 'get_fields_and_field_types'):
                    result[field] = [self._msg_to_dict(v) for v in value]
                else:
                    result[field] = list(value)
            elif isinstance(value, bytes):
                result[field] = base64.b64encode(value).decode('utf-8')
            else:
                result[field] = value
        
        return result
    
    async def _broadcast_message(self, topic, msg_type, msg):
        """Broadcast message to all connected WebSocket clients"""
        if not self.ws_clients:
            return
        
        try:
            # Convert message to JSON-compatible format
            data = {
                'op': 'publish',
                'topic': topic,
                'msg': self._msg_to_dict(msg)
            }
            
            message = json.dumps(data, default=str)
            
            # Send to all clients
            disconnected = set()
            for client in self.ws_clients.copy():
                try:
                    await client.send(message)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)
            
            # Remove disconnected clients
            self.ws_clients -= disconnected
            
        except Exception as e:
            self.get_logger().debug(f'Broadcast error: {e}')
    
    async def _handle_client(self, websocket):
        """Handle a WebSocket client connection"""
        self.ws_clients.add(websocket)
        client_addr = websocket.remote_address
        self.get_logger().info(f'Client connected: {client_addr}')
        
        try:
            # Send available topics on connect
            topics_info = {
                'op': 'advertise',
                'topics': [
                    {'topic': '/scan', 'type': 'sensor_msgs/LaserScan'},
                    {'topic': '/odom', 'type': 'nav_msgs/Odometry'},
                    {'topic': '/imu/data_raw', 'type': 'sensor_msgs/Imu'},
                    {'topic': '/battery_state', 'type': 'sensor_msgs/BatteryState'},
                    {'topic': '/map', 'type': 'nav_msgs/OccupancyGrid'},
                    {'topic': '/tf', 'type': 'tf2_msgs/TFMessage'},
                    {'topic': '/tf_static', 'type': 'tf2_msgs/TFMessage'},
                    {'topic': '/camera/color/image_raw', 'type': 'sensor_msgs/Image'},
                ]
            }
            await websocket.send(json.dumps(topics_info))
            
            # Handle incoming messages
            async for message in websocket:
                try:
                    data = json.loads(message)
                    op = data.get('op', '')
                    
                    if op == 'subscribe':
                        topic = data.get('topic', '')
                        self.get_logger().info(f'Client subscribed to: {topic}')
                    
                    elif op == 'publish':
                        # Handle incoming publish (e.g., cmd_vel)
                        topic = data.get('topic', '')
                        if topic == '/cmd_vel':
                            self._publish_cmd_vel(data.get('msg', {}))
                    
                except json.JSONDecodeError:
                    self.get_logger().warn(f'Invalid JSON from client')
        
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(websocket)
            self.get_logger().info(f'Client disconnected: {client_addr}')
    
    def _publish_cmd_vel(self, msg_data):
        """Publish cmd_vel from WebSocket client"""
        if not hasattr(self, 'cmd_vel_pub'):
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        try:
            twist = Twist()
            linear = msg_data.get('linear', {})
            angular = msg_data.get('angular', {})
            
            twist.linear.x = float(linear.get('x', 0))
            twist.linear.y = float(linear.get('y', 0))
            twist.linear.z = float(linear.get('z', 0))
            twist.angular.x = float(angular.get('x', 0))
            twist.angular.y = float(angular.get('y', 0))
            twist.angular.z = float(angular.get('z', 0))
            
            self.cmd_vel_pub.publish(twist)
        except Exception as e:
            self.get_logger().error(f'Error publishing cmd_vel: {e}')
    
    def _run_ws_server(self):
        """Run WebSocket server in separate thread"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        async def serve():
            async with websockets.serve(
                self._handle_client, 
                self.host, 
                self.port,
                ping_interval=30,
                ping_timeout=10
            ):
                await asyncio.Future()  # Run forever
        
        self.loop.run_until_complete(serve())


def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
