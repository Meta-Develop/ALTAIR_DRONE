#!/usr/bin/env python3
"""
ALTAIR DDS Topic Monitor - Standalone Tool

Monitors all active ROS 2 DDS topics and displays real-time information
including topic name, message type, and publication rate.

Usage:
    python dds_monitor.py

Features:
    - Auto-discovers all topics
    - Color-coded rate indicators (Green/Yellow/Red)
    - Real-time Hz calculation for key topics
    - No ros2 launch required
"""
import os
import sys
import time
import subprocess
import threading
from collections import defaultdict

# Set CycloneDDS Config (same as imu_monitor.py)
os.environ['CYCLONEDDS_URI'] = 'file:///d:/home/6.kennkyuu/ALTAIR_DRONE/cyclonedds_pc.xml'

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# ANSI Color Codes
class Colors:
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"
    BOLD = "\033[1m"
    RESET = "\033[0m"

# Key topics to monitor with expected rates
KEY_TOPICS = {
    '/pico/imu_batch': {'expected_hz': 250, 'warn_hz': 100},
    '/pico/imu_raw': {'expected_hz': 1000, 'warn_hz': 500},
    '/pico/esc_telemetry': {'expected_hz': 100, 'warn_hz': 50},
    '/imu/filtered': {'expected_hz': 1000, 'warn_hz': 500},
    '/odometry/predicted': {'expected_hz': 100, 'warn_hz': 50},
    '/odometry/filtered': {'expected_hz': 100, 'warn_hz': 50},
    '/cmd_vel': {'expected_hz': 10, 'warn_hz': 5},
    '/control/actuator_commands': {'expected_hz': 250, 'warn_hz': 100},
}

class DDSMonitor(Node):
    def __init__(self):
        super().__init__('dds_monitor')
        
        # Track message counts per topic
        self.topic_counts = defaultdict(int)
        self.topic_last_time = defaultdict(float)
        self.topic_rates = defaultdict(float)
        self.subscriptions = {}
        
        # QoS for sensor data (best effort)
        self.qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Timer for rate calculation and display (every 1 second)
        self.display_timer = self.create_timer(1.0, self.display_callback)
        
        # Timer for topic discovery (every 5 seconds)
        self.discovery_timer = self.create_timer(5.0, self.discovery_callback)
        
        # Initial discovery
        self.discovery_callback()
        
        self.print_header()
    
    def print_header(self):
        print("\n" + "=" * 70)
        print(f"{Colors.CYAN}{Colors.BOLD}  ALTAIR DDS Topic Monitor{Colors.RESET}")
        print("=" * 70)
        print(f"  {Colors.GREEN}■{Colors.RESET} Good Rate  "
              f"{Colors.YELLOW}■{Colors.RESET} Warning  "
              f"{Colors.RED}■{Colors.RESET} Low/No Data")
        print("=" * 70 + "\n")
    
    def discovery_callback(self):
        """Discover new topics and subscribe to key ones."""
        try:
            # Get list of topics
            topic_names_and_types = self.get_topic_names_and_types()
            
            for topic_name, types in topic_names_and_types:
                # Skip internal topics
                if topic_name.startswith('/rosout') or topic_name.startswith('/parameter'):
                    continue
                
                # Subscribe to key topics if not already
                if topic_name in KEY_TOPICS and topic_name not in self.subscriptions:
                    self.subscribe_to_topic(topic_name, types[0])
                    
        except Exception as e:
            self.get_logger().warn(f"Discovery error: {e}")
    
    def subscribe_to_topic(self, topic_name, msg_type_str):
        """Create a generic subscription to count messages."""
        try:
            # Import message type dynamically
            msg_module, msg_class = self.get_message_class(msg_type_str)
            if msg_module is None:
                return
            
            # Create callback with topic name captured
            def callback(msg, topic=topic_name):
                self.topic_counts[topic] += 1
            
            sub = self.create_subscription(msg_module, topic_name, callback, self.qos_sensor)
            self.subscriptions[topic_name] = sub
            self.topic_last_time[topic_name] = time.time()
            
        except Exception as e:
            self.get_logger().debug(f"Could not subscribe to {topic_name}: {e}")
    
    def get_message_class(self, type_str):
        """Dynamically import message class from type string like 'sensor_msgs/msg/Imu'."""
        try:
            parts = type_str.split('/')
            if len(parts) != 3:
                return None, None
            
            package, _, class_name = parts
            module = __import__(f"{package}.msg", fromlist=[class_name])
            return getattr(module, class_name), class_name
        except Exception:
            return None, None
    
    def display_callback(self):
        """Calculate rates and display status."""
        now = time.time()
        
        # Clear screen (Windows compatible)
        if os.name == 'nt':
            os.system('cls')
        else:
            os.system('clear')
        
        self.print_header()
        
        # Get all topics
        topic_names_and_types = self.get_topic_names_and_types()
        
        # Print key topics first
        print(f"{Colors.BOLD}Key Topics:{Colors.RESET}")
        print("-" * 70)
        
        for topic_name, config in KEY_TOPICS.items():
            # Calculate rate
            count = self.topic_counts.get(topic_name, 0)
            elapsed = now - self.topic_last_time.get(topic_name, now)
            rate = count / elapsed if elapsed > 0 else 0
            
            # Reset count
            self.topic_counts[topic_name] = 0
            self.topic_last_time[topic_name] = now
            
            # Determine color
            if rate >= config['expected_hz'] * 0.8:
                color = Colors.GREEN
                status = "●"
            elif rate >= config['warn_hz']:
                color = Colors.YELLOW
                status = "●"
            else:
                color = Colors.RED
                status = "○" if rate == 0 else "●"
            
            # Format output
            rate_str = f"{rate:6.1f} Hz" if rate > 0 else "  ---- "
            print(f"  {color}{status}{Colors.RESET} {topic_name:<40} {color}{rate_str}{Colors.RESET}")
        
        print()
        
        # Print other discovered topics
        other_topics = []
        for topic_name, types in topic_names_and_types:
            if topic_name not in KEY_TOPICS:
                if not topic_name.startswith('/rosout') and not topic_name.startswith('/parameter'):
                    other_topics.append((topic_name, types[0] if types else "unknown"))
        
        if other_topics:
            print(f"{Colors.BOLD}Other Topics ({len(other_topics)}):{Colors.RESET}")
            print("-" * 70)
            for topic_name, msg_type in other_topics[:10]:  # Limit to 10
                print(f"  {Colors.WHITE}○{Colors.RESET} {topic_name:<40} [{msg_type}]")
            if len(other_topics) > 10:
                print(f"  ... and {len(other_topics) - 10} more")
        
        print()
        print(f"[Press Ctrl+C to exit] Last update: {time.strftime('%H:%M:%S')}")

def main():
    rclpy.init()
    
    print("Starting ALTAIR DDS Monitor...")
    print("Discovering topics...")
    
    node = DDSMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down DDS Monitor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
