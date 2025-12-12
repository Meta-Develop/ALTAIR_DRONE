#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
import sys

def main():
    rclpy.init()
    node = rclpy.create_node('imu_test_sub')
    received = [0]
    data_sample = [None]
    
    def callback(msg):
        received[0] += 1
        if data_sample[0] is None:
            data_sample[0] = list(msg.data[:6])
    
    # Use Best Effort QoS to match Micro-ROS publisher
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )
    
    sub = node.create_subscription(Float32MultiArray, '/pico/imu_batch', callback, qos)
    
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.25)
    
    if received[0] > 0:
        print(f"SUCCESS: Received {received[0]} messages")
        print(f"Sample data (first 6): {data_sample[0]}")
    else:
        print("FAILURE: No messages received")
    
    rclpy.shutdown()
    sys.exit(0 if received[0] > 0 else 1)

if __name__ == '__main__':
    main()
