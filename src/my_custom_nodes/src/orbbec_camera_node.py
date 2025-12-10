#!/usr/bin/env python3
"""
Orbbec Camera ROS2 Node
Publishes RGB and Depth images from Yahboom AI View (Orbbec Gemini) camera
Uses OpenCV for capture, with proper frame IDs for ROS2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from threading import Thread, Lock


class OrbbecCameraNode(Node):
    def __init__(self):
        super().__init__('orbbec_camera_node')
        
        # Parameters
        self.declare_parameter('rgb_device', 0)
        self.declare_parameter('depth_device', 1)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('rgb_width', 640)
        self.declare_parameter('rgb_height', 480)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('rgb_optical_frame', 'camera_rgb_optical_frame')
        self.declare_parameter('depth_optical_frame', 'camera_depth_optical_frame')
        
        # Get parameters
        self.rgb_device = self.get_parameter('rgb_device').value
        self.depth_device = self.get_parameter('depth_device').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.rgb_width = self.get_parameter('rgb_width').value
        self.rgb_height = self.get_parameter('rgb_height').value
        self.depth_width = self.get_parameter('depth_width').value
        self.depth_height = self.get_parameter('depth_height').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.rgb_optical_frame = self.get_parameter('rgb_optical_frame').value
        self.depth_optical_frame = self.get_parameter('depth_optical_frame').value
        
        # Publishers
        self.rgb_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, 'camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, 'camera/depth/camera_info', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera capture objects
        self.rgb_cap = None
        self.depth_cap = None
        self.lock = Lock()
        self.running = True
        
        # Initialize cameras
        self._init_cameras()
        
        # Create camera info messages
        self.rgb_camera_info = self._create_camera_info(self.rgb_width, self.rgb_height, self.rgb_optical_frame)
        self.depth_camera_info = self._create_camera_info(self.depth_width, self.depth_height, self.depth_optical_frame)
        
        # Timer for publishing
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.publish_frames)
        
        self.get_logger().info(f'Orbbec camera node started - RGB: /dev/video{self.rgb_device}, Depth: /dev/video{self.depth_device}')
    
    def _init_cameras(self):
        """Initialize camera capture objects"""
        try:
            self.rgb_cap = cv2.VideoCapture(self.rgb_device)
            if self.rgb_cap.isOpened():
                self.rgb_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.rgb_width)
                self.rgb_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.rgb_height)
                self.rgb_cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
                self.get_logger().info(f'RGB camera opened: /dev/video{self.rgb_device}')
            else:
                self.get_logger().warn(f'Failed to open RGB camera: /dev/video{self.rgb_device}')
        except Exception as e:
            self.get_logger().error(f'RGB camera init error: {e}')
        
        try:
            self.depth_cap = cv2.VideoCapture(self.depth_device)
            if self.depth_cap.isOpened():
                self.depth_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.depth_width)
                self.depth_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.depth_height)
                self.depth_cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
                self.get_logger().info(f'Depth camera opened: /dev/video{self.depth_device}')
            else:
                self.get_logger().warn(f'Failed to open depth camera: /dev/video{self.depth_device}')
        except Exception as e:
            self.get_logger().error(f'Depth camera init error: {e}')
    
    def _create_camera_info(self, width, height, frame_id):
        """Create a basic CameraInfo message"""
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width = width
        info.height = height
        
        # Default camera matrix (rough estimate - should be calibrated)
        fx = width * 1.0  # Approximate focal length
        fy = height * 1.0
        cx = width / 2.0
        cy = height / 2.0
        
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion assumed
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        info.distortion_model = 'plumb_bob'
        
        return info
    
    def publish_frames(self):
        """Publish camera frames"""
        now = self.get_clock().now().to_msg()
        
        with self.lock:
            # Publish RGB frame
            if self.rgb_cap is not None and self.rgb_cap.isOpened():
                ret, rgb_frame = self.rgb_cap.read()
                if ret:
                    # Convert BGR to RGB
                    rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB)
                    
                    # Create and publish Image message
                    rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='rgb8')
                    rgb_msg.header.stamp = now
                    rgb_msg.header.frame_id = self.rgb_optical_frame
                    self.rgb_pub.publish(rgb_msg)
                    
                    # Publish camera info
                    self.rgb_camera_info.header.stamp = now
                    self.rgb_info_pub.publish(self.rgb_camera_info)
            
            # Publish Depth frame
            if self.depth_cap is not None and self.depth_cap.isOpened():
                ret, depth_frame = self.depth_cap.read()
                if ret:
                    # Convert to grayscale if needed (depth is usually mono)
                    if len(depth_frame.shape) == 3:
                        depth_frame = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)
                    
                    # Convert to 16-bit depth (millimeters)
                    # Scale 8-bit to approximate depth in mm
                    depth_16bit = depth_frame.astype(np.uint16) * 40  # Scale factor
                    
                    # Create and publish Image message
                    depth_msg = self.bridge.cv2_to_imgmsg(depth_16bit, encoding='16UC1')
                    depth_msg.header.stamp = now
                    depth_msg.header.frame_id = self.depth_optical_frame
                    self.depth_pub.publish(depth_msg)
                    
                    # Publish camera info
                    self.depth_camera_info.header.stamp = now
                    self.depth_info_pub.publish(self.depth_camera_info)
    
    def destroy_node(self):
        """Clean up resources"""
        self.running = False
        with self.lock:
            if self.rgb_cap is not None:
                self.rgb_cap.release()
            if self.depth_cap is not None:
                self.depth_cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OrbbecCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
