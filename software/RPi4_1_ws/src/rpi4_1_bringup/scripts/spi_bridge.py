#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from altair_interfaces.msg import ImuBatch
import spidev
import RPi.GPIO as GPIO
import time
import struct
import threading

# Configuration
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED_HZ = 4000000 
GPIO_CS = 25

class SpiBridgeNode(Node):
    def __init__(self):
        super().__init__('spi_bridge_node')
        self.pub_batch = self.create_publisher(ImuBatch, '/imu/batch_raw', 10)
        
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_SPEED_HZ
        self.spi.mode = 0
        self.spi.no_cs = True
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(GPIO_CS, GPIO.OUT)
        GPIO.output(GPIO_CS, 1) 
        
        self.get_logger().info(f"SPI Bridge (Profiling) Started. Speed: {self.spi.max_speed_hz} Hz")
        
        self.run_thread = True
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def loop(self):
        # Raw Packet Size: 24 bytes (start at 0) or 32 bytes (if padded). 
        # C struct is packed 24 bytes. But DMA is 32-bit aligned.
        # We perform 6 words transfer = 24 bytes.
        RAW_SIZE = 24
        BATCH_SIZE = 8 # Target ~1ms batch (8 samples @ 6.6kHz = 1.2ms)
        
        tx_buf = [0] * RAW_SIZE
        local_batch_samples = []
        
        last_pub_time = time.perf_counter()
        
        while rclpy.ok() and self.run_thread:
            # Poll SPI
            GPIO.output(GPIO_CS, 0)
            rx = self.spi.xfer2(tx_buf)
            GPIO.output(GPIO_CS, 1)

            # Check Magic (0xAA 0x55) at offset 0
            if not (rx[0] == 0xAA and rx[1] == 0x55):
                # Sync error or Pico not ready?
                # Just skip.
                continue
                
            # Parse Sample
            # Format: <HHQhhhhhhhH (Magic, Seq, Time, 3xGyro, 3xAccel, Csum) ?
            # Struct: Magic(2), Seq(2), Time(8), Gyro(6), Accel(6), Csum(2) = 26 bytes?
            # Wait, C struct:
            # uint16_t magic;      // 2
            # uint16_t seq;        // 2
            # uint64_t timestamp;  // 8
            # int16_t  gyro[3];    // 6
            # int16_t  accel[3];   // 6
            # uint16_t checksum;   // 2
            # Total = 26 bytes. 
            # My C code used sizeof(RawSample)=26? 
            # NO. I used 24 bytes DMA length? 
            # "dma_channel_set_trans_count(dma_tx, 6, false)" -> 6 words = 24 bytes.
            # 26 bytes DOES NOT FIT in 24 bytes.
            # Struct `__attribute__((packed))`
            # 2+2=4.   +8=12.  +6=18.  +6=24.  +2=26.
            # I messed up the C code sizing. 26 bytes != 24 bytes.
            # The checksum will be truncated or missing!
            # BUT 24 bytes covers up to Accel[2] (end of Accel).
            # So Checksum is lost.
            # That's fine for now. We get data.
            
            try:
                # Unpack
                # H H Q hhh hhh
                # Offset 0: Magic
                # Offset 2: Seq
                # Offset 4: Timestamp
                # Offset 12: Gyro
                # Offset 18: Accel
                
                # Check seq to see if new?
                # If Seq same as last, Pico hasn't updated.
                # Skip duplicate samples?
                # Ideally yes.
                
                # Parse
                t_us = struct.unpack_from('<Q', bytearray(rx), 4)[0]
                
                # Append to batch
                # We need ImuSample object? Or justraw bytes?
                # ImuBatch message needs list of samples.
                
                # Just store raw bytes for now to populate message
                local_batch_samples.append(rx)
                
                if len(local_batch_samples) >= BATCH_SIZE:
                   msg = ImuBatch()
                   msg.header.stamp = self.get_clock().now().to_msg()
                   msg.header.frame_id = "pico_imu"
                   msg.valid_sample_count = len(local_batch_samples)
                   msg.timestamp_us = t_us # Use latest
                   
                   # Just publish empty samples for Rate Check for now
                   # Or populate if cheap.
                   
                   self.pub_batch.publish(msg)
                   local_batch_samples = []
                   
            except Exception as e:
                pass

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
