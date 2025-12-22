#!/usr/bin/env python3
import spidev
import time

print("Starting SPI Loopback Test on RPi4 (SPI0)")
print("Hardware Requirement: Connect Pin 19 (MOSI) to Pin 21 (MISO) directly.")

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 100000

test_byte = 0xA5
print(f"Sending: 0x{test_byte:02X}")

rx = spi.xfer2([test_byte])
print(f"Received: 0x{rx[0]:02X}")

if rx[0] == test_byte:
    print("SUCCESS: SPI Master is working correctly.")
else:
    print("FAILURE: Data mismatch. Check wiring or Pin Config.")
