#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import spidev
import struct
import time
import os

# --- Sysfs Manual CS ---
class SysfsCS:
    def __init__(self, pin): 
        self.pin = pin
        self.path = f"/sys/class/gpio/gpio{pin}/value"
        self.fd = None
        try:
            self.fd = open(self.path, 'w')
            self.deselect()
        except Exception as e:
            print(f"Failed to open {self.path}: {e}")
            print("Did you run setup_gpio.sh with sudo?")

    def select(self):
        if self.fd:
            self.fd.write('0')
            self.fd.flush()

    def deselect(self):
        if self.fd:
            self.fd.write('1')
            self.fd.flush()

    def __del__(self):
        if self.fd: self.fd.close()

class SpiBridgeNode(Node):
    def __init__(self):
        super().__init__('spi_bridge_node')
        
        # Publishers
        self.imu_pub = self.create_publisher(Float32MultiArray, '/pico/imu_batch', 10)
        self.esc_pub = self.create_publisher(Float32MultiArray, '/pico/esc_telemetry', 10)
        self.mag_pub = self.create_publisher(Float32MultiArray, '/pico/mag', 10)
        
        # Subscriptions
        self.sub_esc = self.create_subscription(
            Float32MultiArray, '/control/esc_commands', self.esc_callback, 10
        )
        self.sub_override = self.create_subscription(
            Float32MultiArray, '/control/manual_override', self.override_callback, 10
        )

        self.current_commands = [0.0] * 12
        self.manual_override_active = False 

        # --- Sensor Setup (SPI0 + Manual CS Pin 29) ---
        self.spi_sensors = spidev.SpiDev()
        self.cs_sensors = None
        try:
            self.cs_sensors = SysfsCS(pin=5) # GPIO 5 = Pin 29
            self.spi_sensors.open(0, 0) 
            self.spi_sensors.max_speed_hz = 1000 # 1kHz for extreme timing debug
            self.spi_sensors.mode = 0 
            self.spi_sensors.no_cs = True 
            self.get_logger().info("SPI Sensors (SPI0 + Manual CS Pin 29) Initialized @ 1kHz")
        except Exception as e:
            self.get_logger().error(f"Failed to open SPI Sensors: {e}")

        # --- Actuator Setup (SPI1 + Manual CS Pin 36) ---
        self.spi_actuators = spidev.SpiDev()
        self.cs_actuators = None
        try:
            self.cs_actuators = SysfsCS(pin=16) # GPIO 16 = Pin 36
            self.spi_actuators.open(1, 0) 
            self.spi_actuators.max_speed_hz = 1000000
            self.spi_actuators.mode = 0 
            self.spi_actuators.no_cs = True 
            self.get_logger().info("SPI Actuators (SPI1 + Manual CS Pin 36) Initialized")
        except Exception as e:
            self.get_logger().error(f"Failed to open SPI Actuators: {e}")

        self.timer = self.create_timer(0.001, self.timer_callback)

    def esc_callback(self, msg):
        if not self.manual_override_active:
            if len(msg.data) == 6:
                self.current_commands = list(msg.data) + [0.0]*6
            elif len(msg.data) == 12:
                self.current_commands = list(msg.data)

    def override_callback(self, msg):
        if len(msg.data) >= 12:
            self.manual_override_active = True 
            self.current_commands = list(msg.data[:12])

    def shift_buf(self, buf, shift):
        out = bytearray(len(buf))
        carry = 0
        for i in range(len(buf)-1, -1, -1):
            val = buf[i]
            new_val = ((val << shift) | carry) & 0xFF
            carry = (val >> (8 - shift))
            out[i] = new_val
        return out

    def timer_callback(self):
        PAYLOAD_SIZE = 96
        READ_SIZE = PAYLOAD_SIZE * 2
        
        # --- Node A: Sensors (SPI0) ---
        if self.spi_sensors and self.cs_sensors:
            try:
                self.cs_sensors.select()
                # Give slave time to detect CS edge and pre-fill TX FIFO
                time.sleep(0.001)  # 1ms setup delay
                
                resp_s = self.spi_sensors.xfer2([0] * READ_SIZE)
                self.cs_sensors.deselect()
                
                raw_bytes = bytearray(resp_s)
                
                # Sync Search
                header = b'\xAA\xBB\xCC\xDD'
                aligned_bytes = None
                
                idx = raw_bytes.find(header)
                if idx != -1:
                    aligned_bytes = raw_bytes[idx : idx+PAYLOAD_SIZE]
                
                if aligned_bytes is None:
                    shifted = self.shift_buf(raw_bytes, 1)
                    idx = shifted.find(header)
                    if idx != -1:
                        aligned_bytes = shifted[idx : idx+PAYLOAD_SIZE]
                
                if aligned_bytes is not None and len(aligned_bytes) == PAYLOAD_SIZE:
                    floats_s = struct.unpack('<24f', aligned_bytes)
                    # New layout: [0]=counter, [1]=temp, [2-4]=accel, [5-7]=gyro, [8-10]=mag
                    imu_data = floats_s[2:8]  # accel[3] + gyro[3]
                    mag_data = floats_s[8:11]
                    combined_data = list(imu_data) + list(mag_data)

                    msg_imu = Float32MultiArray()
                    msg_imu.data = combined_data
                    self.imu_pub.publish(msg_imu)
                    
                    if any(v != 0.0 for v in mag_data):
                        msg_mag = Float32MultiArray()
                        msg_mag.data = list(mag_data)
                        self.mag_pub.publish(msg_mag)
                else:
                    self.get_logger().warn(f"Sync Fail. RX: {raw_bytes[:32].hex()}", throttle_duration_sec=1.0)
            except Exception as e:
                pass

        # --- Node B: Actuators (SPI1) ---
        if self.spi_actuators and self.cs_actuators:
            try:
                cmd_payload = [0.0] * 24
                if len(self.current_commands) < 12:
                     self.current_commands.extend([0.0] * (12 - len(self.current_commands)))
                cmd_payload[0:12] = self.current_commands[0:12]
                tx_list = list(struct.pack('<24f', *cmd_payload))

                self.cs_actuators.select()
                resp_a = self.spi_actuators.xfer2(tx_list)
                self.cs_actuators.deselect()
            except Exception as e:
                pass
    
    def destroy_node(self):
        if self.spi_sensors: self.spi_sensors.close()
        if self.spi_actuators: self.spi_actuators.close()
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
