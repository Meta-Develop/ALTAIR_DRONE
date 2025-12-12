#!/usr/bin/env python3
"""
Standalone Raw IMU Monitor for ALTAIR GCS (No ROS 2 Launch Required)

This script directly subscribes to /pico/imu_raw via rclpy and prints
the data to the console. It can be run without ros2 launch.

Usage:
    python imu_monitor.py
"""
import os
import sys
import time

# Set CycloneDDS Config
os.environ['CYCLONEDDS_URI'] = 'file:///d:/home/6.kennkyuu/ALTAIR_DRONE/cyclonedds_pc.xml'

# Add install path to Python path
install_path = os.path.join(os.path.dirname(__file__), 'install', 'altair_gcs_gui', 'lib', 'python3.12', 'site-packages')
sys.path.insert(0, install_path)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

class IMUMonitor(Node):
    def __init__(self):
        super().__init__('imu_monitor')
        
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.packet_count = 0
        self.last_time = time.time()
        
        # Subscribe to both raw and batch
        self.sub_imu_raw = self.create_subscription(
            Imu, '/pico/imu_raw', self.imu_raw_callback, qos_sensor)
        
        self.sub_imu_batch = self.create_subscription(
            Float32MultiArray, '/pico/imu_batch', self.imu_batch_callback, qos_sensor)
        
        # Timer for Hz calculation (1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("IMU Monitor Started. Listening on /pico/imu_raw and /pico/imu_batch...")
        print("\n" + "="*60)
        print("ALTAIR IMU Monitor - Listening for DDS Data")
        print("="*60 + "\n")

    def imu_raw_callback(self, msg):
        self.packet_count += 1
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        
        # Print every 100th packet to avoid console flood
        if self.packet_count % 100 == 0:
            print(f"[IMU_RAW] Ax:{ax:7.3f} Ay:{ay:7.3f} Az:{az:7.3f} | Gx:{gx:7.3f} Gy:{gy:7.3f} Gz:{gz:7.3f}")

    def imu_batch_callback(self, msg):
        if len(msg.data) > 0:
            print(f"[IMU_BATCH] Received {len(msg.data) // 6} samples. First: Ax={msg.data[0]:.2f}")

    def timer_callback(self):
        now = time.time()
        elapsed = now - self.last_time
        hz = self.packet_count / elapsed if elapsed > 0 else 0
        
        if hz > 0:
            color = "\033[92m" if hz >= 800 else ("\033[93m" if hz >= 100 else "\033[91m")
            print(f"{color}[RATE] IMU Raw: {hz:.1f} Hz (Packets: {self.packet_count})\033[0m")
        else:
            print(f"\033[91m[RATE] IMU Raw: 0 Hz - No data received!\033[0m")
        
        self.packet_count = 0
        self.last_time = now

def main():
    rclpy.init()
    node = IMUMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
