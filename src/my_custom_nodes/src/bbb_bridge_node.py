#!/usr/bin/env python3
"""
ROS2 BeagleBone Blue Bridge Node

This node communicates with the BeagleBone Blue motor controller via UART
and provides ROS2 interfaces for:
- cmd_vel subscription (Twist) -> motor commands
- odometry publishing (Odometry)
- IMU publishing (Imu)
- battery state publishing (BatteryState)

Mecanum wheel kinematics for 4-wheel drive
"""

import json
import math
import threading
import time
from queue import Empty, Queue

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu
from tf2_ros import TransformBroadcaster


class BBBBridgeNode(Node):
    def __init__(self):
        super().__init__("bbb_bridge")

        # Parameters
        self.declare_parameter("serial_port", "/dev/ttyAMA0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("wheel_base", 0.2)  # Distance between left and right wheels (m)
        self.declare_parameter("wheel_track", 0.15)  # Distance between front and rear wheels (m)
        self.declare_parameter("wheel_radius", 0.05)  # Wheel radius (m)
        self.declare_parameter("encoder_ticks_per_rev", 1440)  # Encoder resolution
        self.declare_parameter("max_motor_speed", 1.0)  # Max duty cycle
        self.declare_parameter("publish_tf", True)

        # Get parameters
        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = self.get_parameter("baud_rate").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.wheel_track = self.get_parameter("wheel_track").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.encoder_ticks_per_rev = self.get_parameter("encoder_ticks_per_rev").value
        self.max_motor_speed = self.get_parameter("max_motor_speed").value
        self.publish_tf = self.get_parameter("publish_tf").value

        # Mecanum kinematics constants
        self.lx_ly = (self.wheel_base + self.wheel_track) / 2.0

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_enc = [0, 0, 0, 0]
        self.first_enc = True
        self.last_time = self.get_clock().now()

        # Serial setup
        self.ser = None
        self.serial_lock = threading.Lock()
        self.cmd_queue = Queue()

        # Publishers - use RELIABLE QoS for compatibility with SLAM and Foxglove
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, "odom", qos)
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", qos)
        self.battery_pub = self.create_publisher(BatteryState, "battery_state", 10)

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Connect to BBB
        self.connect_serial()

        # Start serial reader thread
        self.running = True
        self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
        self.serial_thread.start()

        # Timer for sending motor commands from queue
        self.cmd_timer = self.create_timer(0.02, self.process_cmd_queue)  # 50Hz

        # Timer for publishing TF and odom continuously (required for SLAM)
        if self.publish_tf:
            self.tf_timer = self.create_timer(0.05, self.publish_tf_callback)  # 20Hz

        # Watchdog timer - stop motors if no cmd_vel received
        self.last_cmd_time = time.time()
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)

        self.get_logger().info(f"BBB Bridge Node started on {self.serial_port}")

    def connect_serial(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f"Connected to {self.serial_port}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return False

    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to motor commands using mecanum kinematics"""
        self.last_cmd_time = time.time()

        vx = msg.linear.x  # Forward/backward
        vy = msg.linear.y  # Left/right strafe
        wz = msg.angular.z  # Rotation

        # Mecanum inverse kinematics
        # Motor layout (top view):
        #   M1 --- M2
        #    |     |
        #   M3 --- M4
        # Positive = forward

        # Calculate wheel velocities
        m1 = vx - vy - self.lx_ly * wz  # Front left
        m2 = vx + vy + self.lx_ly * wz  # Front right
        m3 = vx + vy - self.lx_ly * wz  # Rear left
        m4 = vx - vy + self.lx_ly * wz  # Rear right

        # Normalize to max speed
        max_val = max(abs(m1), abs(m2), abs(m3), abs(m4), 1.0)
        if max_val > self.max_motor_speed:
            scale = self.max_motor_speed / max_val
            m1 *= scale
            m2 *= scale
            m3 *= scale
            m4 *= scale

        # Queue motor command
        cmd = {"cmd": "motors", "m1": m1, "m2": m2, "m3": m3, "m4": m4}
        self.cmd_queue.put(cmd)

    def process_cmd_queue(self):
        """Send queued commands to BBB"""
        try:
            cmd = self.cmd_queue.get_nowait()
            self.send_command(cmd)
        except Empty:
            pass

    def send_command(self, cmd):
        """Send a command to BBB"""
        if self.ser and self.ser.is_open:
            try:
                with self.serial_lock:
                    msg = json.dumps(cmd) + "\n"
                    self.ser.write(msg.encode())
                    self.ser.flush()
            except Exception as e:
                self.get_logger().error(f"Send error: {e}")

    def watchdog_callback(self):
        """Stop motors if no commands received recently"""
        if time.time() - self.last_cmd_time > 0.5:
            self.send_command({"cmd": "stop"})

    def publish_tf_callback(self):
        """Publish TF and odometry continuously for SLAM"""
        now = self.get_clock().now()

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

    def serial_reader(self):
        """Thread to read serial data from BBB"""
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting > 0:
                        with self.serial_lock:
                            line = self.ser.readline().decode("utf-8", errors="ignore")
                        if line.strip():
                            self.process_sensor_data(line)
                except Exception as e:
                    self.get_logger().error(f"Read error: {e}")
            time.sleep(0.001)

    def process_sensor_data(self, data_str):
        """Process sensor data from BBB"""
        try:
            data = json.loads(data_str.strip())

            if data.get("type") == "sensors":
                self.update_odometry(data.get("enc", [0, 0, 0, 0]))
                self.publish_imu(data.get("imu", {}))
                self.publish_battery(data.get("battery", 0))

            elif data.get("type") == "error":
                self.get_logger().warn(f"BBB error: {data.get('msg')}")

        except json.JSONDecodeError:
            pass
        except Exception as e:
            self.get_logger().error(f"Process error: {e}")

    def update_odometry(self, encoders):
        """Calculate odometry from encoder values"""
        if self.first_enc:
            self.last_enc = encoders
            self.first_enc = False
            return

        # Calculate encoder deltas
        d_enc = [encoders[i] - self.last_enc[i] for i in range(4)]
        self.last_enc = encoders

        # Convert to wheel displacements (meters)
        meters_per_tick = (2 * math.pi * self.wheel_radius) / self.encoder_ticks_per_rev
        d_wheels = [d * meters_per_tick for d in d_enc]

        # Mecanum forward kinematics
        # Average wheel contributions
        vx = (d_wheels[0] + d_wheels[1] + d_wheels[2] + d_wheels[3]) / 4.0
        vy = (-d_wheels[0] + d_wheels[1] + d_wheels[2] - d_wheels[3]) / 4.0
        wz = (-d_wheels[0] + d_wheels[1] - d_wheels[2] + d_wheels[3]) / (4.0 * self.lx_ly)

        # Update pose
        # Use rotation matrix for proper integration
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)

        self.x += vx * cos_theta - vy * sin_theta
        self.y += vx * sin_theta + vy * cos_theta
        self.theta += wz

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish odometry
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Quaternion from yaw
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocity
        if dt > 0:
            odom.twist.twist.linear.x = vx / dt
            odom.twist.twist.linear.y = vy / dt
            odom.twist.twist.angular.z = wz / dt

        self.odom_pub.publish(odom)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = math.sin(self.theta / 2.0)
            t.transform.rotation.w = math.cos(self.theta / 2.0)
            self.tf_broadcaster.sendTransform(t)

    def publish_imu(self, imu_data):
        """Publish IMU data"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Linear acceleration (m/s^2) - ensure float
        msg.linear_acceleration.x = float(imu_data.get("ax", 0.0))
        msg.linear_acceleration.y = float(imu_data.get("ay", 0.0))
        msg.linear_acceleration.z = float(imu_data.get("az", 0.0))

        # Angular velocity (rad/s) - convert from deg/s, ensure float
        msg.angular_velocity.x = math.radians(float(imu_data.get("gx", 0.0)))
        msg.angular_velocity.y = math.radians(float(imu_data.get("gy", 0.0)))
        msg.angular_velocity.z = math.radians(float(imu_data.get("gz", 0.0)))

        # Orientation not provided by raw IMU
        msg.orientation_covariance[0] = -1  # Orientation not available

        self.imu_pub.publish(msg)

    def publish_battery(self, voltage):
        """Publish battery state"""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(voltage)
        msg.present = voltage > 5.0

        # Estimate percentage for 3S LiPo (9.0V empty, 12.6V full)
        if voltage > 9.0:
            msg.percentage = min(1.0, (voltage - 9.0) / 3.6)
        else:
            msg.percentage = 0.0

        self.battery_pub.publish(msg)

    def destroy_node(self):
        """Cleanup"""
        self.running = False
        self.send_command({"cmd": "stop"})
        if self.ser:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BBBBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
