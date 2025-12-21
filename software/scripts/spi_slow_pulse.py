#!/usr/bin/env python3
import spidev
import time

print("Testing Ultra Slow SPI (1kHz) for Visibility...")
print("CS Pulse should be 0.8 seconds long.")

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000 # 1kHz

for i in range(5):
    print(f"Pulse {i+1}/5...")
    # Send 100 bytes. At 1kHz (1ms/bit), 800 bits = 800ms.
    spi.xfer2([0xAA]*100) 
    time.sleep(1.0)

print("Done.")
