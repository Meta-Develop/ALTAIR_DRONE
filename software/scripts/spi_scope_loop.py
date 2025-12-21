#!/usr/bin/env python3
import spidev
import time

print("SPI Scope Loop Started.")
print("Generating 10Hz CS Square Wave on CE0 (GPIO 8 / Pin 24).")
print("High (50ms) -> Low (50ms) -> Repeat")

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 100000 # 100kHz

# 50ms Low duration requires sending bytes.
# 1 byte @ 100kHz = 80us.
# 50ms / 80us = 625 bytes.
data = [0xAA] * 625

try:
    while True:
        # CS goes LOW here
        spi.xfer2(data) 
        # CS goes HIGH after xfer returns
        
        time.sleep(0.05) # 50ms High
except KeyboardInterrupt:
    pass
