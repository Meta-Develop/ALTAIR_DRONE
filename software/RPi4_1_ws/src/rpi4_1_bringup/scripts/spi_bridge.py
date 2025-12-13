#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import spidev
import struct
import time

class SpiBridgeNode(Node):
    def __init__(self):
        super().__init__('spi_bridge_node')
        
        # Publisher for IMU Batch (to be consumed by sensor_processing_node)
        # Publishing raw floats: [ax, ay, az, gx, gy, gz]
        self.imu_pub = self.create_publisher(Float32MultiArray, '/pico/imu_batch', 10)
        
        # SPI Setup
        self.spi = spidev.SpiDev()
        try:
            self.spi.open(0, 0) # Bus 0, Device 0 (CE0 / Pin 24)
            self.spi.max_speed_hz = 4000000 # 4MHz to match Pico
            self.spi.mode = 0 # CPOL=0, CPHA=0 (Matches Pico)
            self.get_logger().info("SPI Initialized: Bus 0, Device 0 (CE0 / Pin 24), 4MHz")
        except Exception as e:
            self.get_logger().error(f"Failed to open SPI: {e}")
            # We continue (loop will fail) or exit? Exit is better.
            raise e

        # Timer (Polling Pico)
        # Target: 250Hz check? 
        # Note: Pico updates buffer at 1kHz. 
        # Without synchronization/interrupt, we will miss samples or duplicate them.
        # But purely for "Debugging DDS" flow, polling is sufficient start.
        self.timer = self.create_timer(0.004, self.timer_callback) # 250Hz (4ms)

    def timer_callback(self):
        # Transaction: Read 32 bytes (8 floats)
        # Pico sends: [TimeSec, TimeUs, Ax, Ay, Az, Gx, Gy, Gz]
        try:
            # Transfer 32 dummy bytes to clock out data from Slave
            resp = self.spi.xfer2([0] * 32)
            
            # Pack bytes into bytearray
            raw_bytes = bytearray(resp)
            
            # Unpack floats (Little Endian <, 8 floats f)
            floats = struct.unpack('<8f', raw_bytes)
            
            # Extract IMU part (Indexes 2-7) and Time (for debug if needed)
            # t_sec = floats[0]
            # t_us = floats[1]
            imu_data = floats[2:8] # [ax, ay, az, gx, gy, gz]
            
            # Simple check for valid data (reject all zeros which might be SPI fail)
            if all(v == 0.0 for v in imu_data):
                # Optionally warn, but 0.0 is valid for stationary gyro/accel? 
                # Probably not exact 0.00000.
                pass

            msg = Float32MultiArray()
            msg.data = list(imu_data)
            
            self.imu_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().warn(f"SPI Read Failed: {e}")

    def destroy_node(self):
        self.spi.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SpiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
