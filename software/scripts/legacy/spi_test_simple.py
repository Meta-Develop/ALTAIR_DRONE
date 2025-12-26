#!/usr/bin/env python3
"""Simple SPI test to verify communication with Pico 2 boards"""
import spidev
import struct
import time

def test_spi(device, name):
    print(f"\n=== Testing {name} on CE{device} ===")
    spi = spidev.SpiDev()
    try:
        spi.open(0, device)
        spi.max_speed_hz = 4000000
        spi.mode = 0
        
        for i in range(3):
            resp = spi.xfer2([0]*96)
            floats = struct.unpack('<24f', bytearray(resp))
            
            # Check if data is non-zero
            non_zero = sum(1 for f in floats if f != 0.0)
            print(f"Read {i+1}: Non-zero floats: {non_zero}/24")
            print(f"  Time: {floats[0]:.0f}, {floats[1]:.0f}")
            print(f"  Accel: ({floats[2]:.2f}, {floats[3]:.2f}, {floats[4]:.2f})")
            print(f"  Gyro: ({floats[5]:.2f}, {floats[6]:.2f}, {floats[7]:.2f})")
            time.sleep(0.1)
            
        spi.close()
        return True
    except Exception as e:
        print(f"Error: {e}")
        return False

if __name__ == "__main__":
    print("SPI Connection Test")
    print("=" * 40)
    
    test_spi(0, "Pico 2 A (Sensors)")
    test_spi(1, "Pico 2 B (Actuators)")
    
    print("\n=== Test Complete ===")
