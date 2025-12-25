#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from altair_interfaces.msg import ImuBatch
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu, MagneticField, FluidPressure, Range
import spidev
import RPi.GPIO as GPIO
import time
import struct
import threading

# Configuration
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED_HZ = 4000000 # Try 4MHz for speed (Python works at 4MHz if synced?)
# verification script worked at 1MHz. test_spi_speed worked 35% at 8MHz blind.
# With proper handshake, 4MHz or 8MHz should work.
# Let's start with 1MHz to be safe, then upgrade.
SPI_SPEED_HZ_SAFE = 1000000

GPIO_CS = 25
GPIO_DRDY = 24

class SpiBridgeNode(Node):
    def __init__(self):
        super().__init__('spi_bridge_node')
        
        # Publishers
        self.pub_batch = self.create_publisher(ImuBatch, '/imu/batch_raw', 10)
        
        # SPI Setup
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_SPEED_HZ # 4MHz
        self.spi.mode = 0
        self.spi.no_cs = True
        
        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(GPIO_CS, GPIO.OUT)
        GPIO.output(GPIO_CS, 1) # CS Idle High
        GPIO.setup(GPIO_DRDY, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
        # Pico drives Output, so Pull-Down doesn't hurt, ensures 0 if float.
        
        self.get_logger().info(f"SPI Bridge (Python) Started. Speed: {self.spi.max_speed_hz} Hz")
        
        # Loop
        self.run_thread = True
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

# Optimized Loop
    def loop(self):
        tx_buf = [0] * 128
        fmt_samples = '<' + 'h' * 48 # 8 samples * 6 axes * 2 bytes = 96 bytes

        # Stats
        last_print = time.time()
        count = 0
        
        while rclpy.ok() and self.run_thread:
            # OPTIMIZED HANDSHAKE
            if GPIO.wait_for_edge(GPIO_DRDY, GPIO.RISING, timeout=5) is None:
                if GPIO.input(GPIO_DRDY): pass
                else: continue

            GPIO.output(GPIO_CS, 0)
            rx = self.spi.xfer2(tx_buf)
            GPIO.output(GPIO_CS, 1)

            # Skip Magic Scan & Unpacking for Speed Test
            # Just publish immediately to verify throughput
            
            # Construct Message (Dummy)
            msg = ImuBatch()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "pico_imu"
            self.pub_batch.publish(msg)

    def destroy_node(self):
        self.run_thread = False
        self.thread.join()
        GPIO.cleanup()
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
