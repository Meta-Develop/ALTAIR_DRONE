#!/usr/bin/env python3
import spidev
import time
import struct

print("SPI Sensor Data Test")
print("Reading ISM330DHCX Data from Pico 2 A (CE0)...")

spi = spidev.SpiDev()
spi.open(0, 0) # CE0
spi.max_speed_hz = 1000000 # 1MHz

# Commands defined in Pico firmware
# CMD_READ_SENSORS = 0x01
CMD_READ = 0x01

for i in range(10):
    # Packet structure: [CMD, 0x00... (padding for response)]
    # Response: [Status, AccelX, Y, Z, GyroX, Y, Z, Temp] (All floats)
    # Total floats = 7. Bytes = 7 * 4 = 28 bytes.
    # Plus status byte? Let's send 32 bytes to be safe.
    
    msg = [CMD_READ] + [0x00]*31
    resp = spi.xfer2(msg)
    
    # Analyze response
    # Skip first byte (dummy/status?)
    # Pico SPI Slave usually shifts out data immediately or with 1 byte delay depending on implementation.
    # Current firmware implementation:
    # spi_write_blocking(spi_default, (uint8_t*)&sensor_data, sizeof(sensor_data));
    # This happens inside the IRQ or transfer loop.
    
    # Let's inspect raw bytes first
    byte_str = " ".join([f"{b:02X}" for b in resp])
    print(f"Raw: {byte_str}")
    
    time.sleep(0.5)

print("Done.")
