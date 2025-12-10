#!/usr/bin/env python3
"""
Auto-save map node for persistent mapping (Roomba-style)
Saves the SLAM map periodically and on shutdown
"""

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph
import os
import time
from datetime import datetime


class AutoSaveMapNode(Node):
    def __init__(self):
        super().__init__('auto_save_map_node')
        
        # Parameters
        self.declare_parameter('save_interval', 60.0)  # Save every 60 seconds
        self.declare_parameter('map_dir', '/home/max/ros2_ws/map')
        self.declare_parameter('map_name', 'persistent_map')
        
        self.save_interval = self.get_parameter('save_interval').value
        self.map_dir = self.get_parameter('map_dir').value
        self.map_name = self.get_parameter('map_name').value
        
        # Ensure map directory exists
        os.makedirs(self.map_dir, exist_ok=True)
        
        # Service clients
        self.save_map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.serialize_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        
        # Wait for services
        self.get_logger().info('Waiting for SLAM toolbox services...')
        self.save_map_client.wait_for_service(timeout_sec=30.0)
        self.serialize_client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('SLAM toolbox services available')
        
        # Timer for periodic saving
        self.timer = self.create_timer(self.save_interval, self.auto_save_callback)
        
        self.get_logger().info(f'Auto-save map node started. Saving every {self.save_interval}s to {self.map_dir}/{self.map_name}')
    
    def auto_save_callback(self):
        """Periodically save the map"""
        self.save_map()
        self.serialize_map()
    
    def save_map(self):
        """Save the map as an image (pgm/yaml)"""
        try:
            req = SaveMap.Request()
            req.name.data = os.path.join(self.map_dir, self.map_name)
            
            future = self.save_map_client.call_async(req)
            # Don't block, just log result when available
            future.add_done_callback(self._save_map_callback)
        except Exception as e:
            self.get_logger().error(f'Error saving map: {e}')
    
    def _save_map_callback(self, future):
        try:
            result = future.result()
            timestamp = datetime.now().strftime('%H:%M:%S')
            self.get_logger().info(f'[{timestamp}] Map saved: {self.map_name}')
        except Exception as e:
            self.get_logger().error(f'Save map failed: {e}')
    
    def serialize_map(self):
        """Serialize the pose graph for later use"""
        try:
            req = SerializePoseGraph.Request()
            req.filename = os.path.join(self.map_dir, f'{self.map_name}_posegraph')
            
            future = self.serialize_client.call_async(req)
            future.add_done_callback(self._serialize_callback)
        except Exception as e:
            self.get_logger().error(f'Error serializing map: {e}')
    
    def _serialize_callback(self, future):
        try:
            result = future.result()
            self.get_logger().debug(f'Pose graph serialized')
        except Exception as e:
            self.get_logger().warn(f'Serialize failed: {e}')
    
    def destroy_node(self):
        """Save map on shutdown"""
        self.get_logger().info('Shutting down - saving final map...')
        self.save_map()
        self.serialize_map()
        time.sleep(1)  # Give time for save to complete
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoSaveMapNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()