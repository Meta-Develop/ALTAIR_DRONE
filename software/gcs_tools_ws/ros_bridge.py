#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import zmq
import json
import time

# ZMQ Setup
ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind("tcp://*:5555")  # Bind to all interfaces (accessible from Windows via localhost if strictly WSL2 localhost forwarding works, or 0.0.0.0)

class ROSBridge(Node):
    def __init__(self):
        super().__init__('ros_zmq_bridge')
        
        # Subscribe to Filtered IMU (High Rate)
        self.create_subscription(Imu, '/imu/filtered', self.imu_callback, 10)
        
        # Subscribe to Raw (for debug)
        self.create_subscription(Float32MultiArray, '/pico/imu_batch', self.batch_callback, 10)
        
        self.get_logger().info("ROS-ZMQ Bridge Started. Publishing on tcp://*:5555")

    def imu_callback(self, msg):
        # Serialize to JSON (fast enough for Viz, binary struct better for very high rate but JSON is easier)
        payload = {
            "topic": "imu",
            "data": {
                "ax": msg.linear_acceleration.x,
                "ay": msg.linear_acceleration.y,
                "az": msg.linear_acceleration.z,
                "gx": msg.angular_velocity.x,
                "gy": msg.angular_velocity.y,
                "gz": msg.angular_velocity.z
            }
        }
        sock.send_string(json.dumps(payload))

    def batch_callback(self, msg):
        # Just notify batch received
        # payload = {"topic": "batch_event", "ts": time.time()}
        # sock.send_string(json.dumps(payload))
        pass

def main():
    rclpy.init()
    node = ROSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
