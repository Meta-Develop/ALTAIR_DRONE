#!/usr/bin/env python3
"""SPI Hex Dump Test"""
import spidev
import time

def dump_spi(device, name):
    print(f"\n=== Testing {name} on CE{device} ===")
    spi = spidev.SpiDev()
    try:
        spi.open(0, device)
        spi.max_speed_hz = 100000 # 100kHz
        spi.mode = 0
        
        print("Reading 32 bytes (Raw Hex):")
        for i in range(5):
            resp = spi.xfer2([0xAA]*32) # Send 0xAA pattern
            hex_str = " ".join([f"{b:02X}" for b in resp])
            print(f"Read {i+1}: {hex_str}")
            time.sleep(0.1)
            
        spi.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    dump_spi(0, "Pico 2 A (Sensors)")
    dump_spi(1, "Pico 2 B (Actuators)")
