#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
import time
import numpy as np

class HardwareTestNode(Node):
    def __init__(self):
        super().__init__('hardware_test_node')
        
        # Publishers
        self.motor_pub = self.create_publisher(Float32MultiArray, '/pico/motor_commands', 10)
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/pico/imu_raw', self.imu_callback, 10)
        self.telem_sub = self.create_subscription(Float32MultiArray, '/pico/esc_telemetry', self.telem_callback, 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        
        # State
        self.imu_received = False
        self.telem_received = False
        self.start_time = time.time()
        
        self.get_logger().info("Hardware Test Node Started. Sending dummy motor commands...")

    def timer_callback(self):
        # Send Dummy Commands (10% throttle)
        msg = Float32MultiArray()
        msg.data = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        self.motor_pub.publish(msg)
        
        # Check timeout
        if time.time() - self.start_time > 5.0 and not self.imu_received:
             self.get_logger().warn("No IMU data received for 5 seconds!")

    def imu_callback(self, msg):
        self.imu_received = True
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        
        self.get_logger().info(f"IMU OK: Accel={accel}, Gyro={gyro}")

    def telem_callback(self, msg):
        self.telem_received = True
        self.get_logger().info(f"Telemetry OK: RPM={msg.data} (Expect 0s)")

def main(args=None):
    rclpy.init(args=args)
    node = HardwareTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
