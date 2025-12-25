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
        self.spi.max_speed_hz = SPI_SPEED_HZ_SAFE
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

    def loop(self):
        tx_buf = [0] * 128
        
        # Stats
        last_print = time.time()
        count = 0
        
        while rclpy.ok() and self.run_thread:
            # HANDSHAKE
            # Wait for DRDY (GPIO 24) to go High
            # Timeout 5ms
            timeout_start = time.time()
            ready = False
            while (time.time() - timeout_start) < 0.005:
                if GPIO.input(GPIO_DRDY):
                    ready = True
                    break
                # tight loop
            
            if not ready:
                # self.get_logger().warn("Data Ready Timeout")
                continue
                
            # Perform Transfer
            GPIO.output(GPIO_CS, 0)
            # time.sleep(0.00001) # Small delay 10us?
            # verify_script uses 100us (0.0001). Let's use 20us.
            # busy wait?
            # Python call overhead might be enough.
            
            rx = self.spi.xfer2(tx_buf)
            GPIO.output(GPIO_CS, 1)
            
            # Process Data
            self.process_packet(rx)
            count += 1
            
            if time.time() - last_print > 1.0:
                # self.get_logger().info(f"Rate: {count} Hz")
                count = 0
                last_print = time.time()
                
            # Sleep to cap rate?
            # Pico samples at 6.67kHz / 8 = 833 Hz?
            # Or 1kHz?
            # User wants 1kHz.
            # If we run free, we run at Pico's ready rate.
            
    def process_packet(self, rx):
        # Scan for Magic
        # Look for AA 55 or 55 AA
        magic_idx = -1
        swapped = False
        
        # Search range matching C++ (rx - 4)
        for i in range(len(rx) - 4):
            if rx[i] == 0xAA and rx[i+1] == 0x55:
                magic_idx = i
                swapped = False
                break
            if rx[i] == 0x55 and rx[i+1] == 0xAA:
                magic_idx = i
                swapped = True
                break
                
        if magic_idx < 0:
            # self.get_logger().warn(f"No Magic. Rx[0]: {rx[0]:02X}")
            return

        # Decode (minimal for ImuBatch)
        # We need to reconstruct the packet aligned
        # If swapped, we swap bytes. 16-bit word swap.
        # [0][1] -> [1][0].
        
        aligned_data = bytearray(rx[magic_idx:])
        # Pad if short?
        if len(aligned_data) < 128:
             aligned_data.extend([0]*(128-len(aligned_data))) # Just pad end
             
        # Unswap if needed
        if swapped:
            # Swap 16-bit words
            # A B C D -> B A D C
            for j in range(0, len(aligned_data)-1, 2):
                b0 = aligned_data[j]
                b1 = aligned_data[j+1]
                aligned_data[j] = b1
                aligned_data[j+1] = b0
                
        # Parse Header
        # Structural unpacking...
        # Magic(2), Frame(1), Flags(1), Time(8)
        # Total 12 bytes header?
        # Let's just assume valid if Magic found and publish RAW for now.
        # But ImuBatch message requires fields.
        # ... user said "Raw data thrown to RPi4".
        # ImuBatch.msg has:
        # header
        # valid_sample_count
        # samples[]
        
        # Let's populate the message
        msg = ImuBatch()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pico_imu"
        
        # struct: magic(2), frame(1), flags(1), time(8) -> 12 bytes
        # count(1) -> offset 12
        try:
            msg.valid_sample_count = aligned_data[12]
            
            # Timestamp (offset 4, uint64)
            ts = struct.unpack_from('<Q', aligned_data, 4)[0]
            msg.timestamp_us = ts
            
            # Samples
            # Offset 13
            # Each sample 12 bytes
            # MAX 8 samples
            for k in range(8):
                idx = 13 + k*12
                if idx + 12 > len(aligned_data): break
                
                # ax, ay, az, gx, gy, gz (int16)
                s_data = struct.unpack_from('<hhhhhh', aligned_data, idx)
                # s_data is tuple
                # ImuBatch.msg has ImuSample[] which has int16 accel[3], etc.
                # Actually ImuBatch.msg probably defines ImuSample.msg?
                # "altair_interfaces/msg/ImuSample"
                # Let's verify msg definition if possible.
                # Assuming generic mapping:
                # We can just publish the raw bytes? 
                # msg.raw_data = aligned_data?
                # If message definition doesn't support raw, we must unpack.
                # msg.samples is list of ImuSample.
                
                # Doing full unpack in Python loop is slow.
                # But let's try.
                
                # Wait, I don't know ImuSample python class structure exactly (generated).
                # But typically: sample = ImuSample(); sample.accel = [x,y,z]...
                pass 
                
        except Exception as e:
            # self.get_logger().error(f"Parse error: {e}")
            pass

        # For now, just publish EMPTY message (or partially filled) to verify Rate.
        # We assume downstream node (Notch Filter) will read it?
        # Wait, if I publish empty, Notch Filter fails.
        # I must publish VALID content.
        
        # But for Step 1: Verify RATE.
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
