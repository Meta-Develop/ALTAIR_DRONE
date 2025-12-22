#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import struct
import time
import math
import sys
import os
import spidev

class PicoBridgeNode(Node):
    def __init__(self):
        super().__init__('pico_bridge_node')
        
        # SPI Configuration (Hardware)
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0) # Bus 0, Device 0
            self.spi.max_speed_hz = 50000 # 50kHz (Matching Loopback Tuning)
            self.spi.mode = 0
            self.get_logger().info("SPI Initialized: /dev/spidev0.0 @ 50kHz")
        except Exception as e:
            self.get_logger().error(f"SPI Init Failed: {e}")
            sys.exit(1)
        
        # Publisher
        self.imu_pub = self.create_publisher(Imu, '/pico/imu', 10)
        
        # Timer (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info("Pico Bridge Node Started (Hardware SPI Mode)")

    def timer_callback(self):
        # Read 33 bytes to allow for bit-shifting recovery
        # (32 bytes packet + 1 byte overlapped/buffer)
        try:
            rx = self.spi.xfer([0] * 33)
            
            # ATTEMPT 1: Check for Direct Match (No Shift)
            if rx[0] == 0xAA and rx[29] == 0xBB:
                self.parse_and_publish(rx)
                return

            # ATTEMPT 2: Check for 2-Bit Right Shift (Latency)
            # Reconstruct into a temporary buffer
            reconstructed = bytearray(32)
            for i in range(32):
                # Shift Left by 2 to counter the Right Shift
                val = ((rx[i] << 2) & 0xFF) | ((rx[i+1] >> 6) & 0x03)
                reconstructed[i] = val
            
            if reconstructed[0] == 0xAA and reconstructed[29] == 0xBB:
                self.parse_and_publish(list(reconstructed))
                return
                
            # ATTEMPT 3: Scan for 0xAA byte (Byte Slip)
            # Only if aligned to byte boundaries
            # if 0xAA in rx: ... (Not implementing unless needed)

            # If we get here, sync failed
            # self.get_logger().warn(f"Sync Fail: Raw[0]={rx[0]:02X}")

        except Exception as e:
            self.get_logger().error(f"SPI Read Error: {e}")

    def parse_and_publish(self, rx):
        try:
            # Protocol: [0xAA, AX, AY, AZ, GX, GY, GZ, TS, 0xBB]
            # Payload is Bytes 1..28 (28 bytes)
            payload = bytes(rx[1:29])
            
            # Struct Unpack: 6 floats + 1 uint32
            data = struct.unpack('<ffffffI', payload)
            
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "pico_link"
            
            # Accelerometer
            msg.linear_acceleration.x = data[0]
            msg.linear_acceleration.y = data[1]
            msg.linear_acceleration.z = data[2]
            
            # Gyroscope
            msg.angular_velocity.x = data[3]
            msg.angular_velocity.y = data[4]
            msg.angular_velocity.z = data[5]
            
            self.imu_pub.publish(msg)
            
            # Debug Log periodically
            # self.get_logger().info(f"IMU: Ax={data[0]:.2f} Az={data[2]:.2f}")
            
        except Exception as e:
            self.get_logger().error(f"Parse Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PicoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'spi'):
            node.spi.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
