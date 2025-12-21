#!/usr/bin/env python3
import spidev
import time

print("Starting CONTINUOUS SPI Loopback Test (Ctrl+C to stop)")
print("Connect MOSI (Pin 19) <=> MISO (Pin 21)")

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 100000
spi.mode = 0b00 # Mode 0

counter = 0

try:
    while True:
        # Send incrementing pattern
        tx = [counter & 0xFF]
        rx = spi.xfer2(tx)
        
        if rx[0] == tx[0]:
            status = "OK"
        else:
            status = "FAIL"
            
        print(f"TX: 0x{tx[0]:02X} | RX: 0x{rx[0]:02X} | {status}")
        
        counter += 1
        time.sleep(0.1)
except KeyboardInterrupt:
    spi.close()
