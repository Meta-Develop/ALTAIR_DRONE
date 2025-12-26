#!/usr/bin/env python3
"""SPI test with slower speed"""
import spidev
import struct
import time

def test_spi(device, name, speed=1000000):
    print(f"\n=== Testing {name} on CE{device} @ {speed//1000}kHz ===")
    spi = spidev.SpiDev()
    try:
        spi.open(0, device)
        spi.max_speed_hz = speed
        spi.mode = 0
        
        for i in range(5):
            # Send dummy bytes to clock in data
            resp = spi.xfer2([0]*96)
            floats = struct.unpack('<24f', bytearray(resp))
            
            # Check if data is non-zero
            non_zero = sum(1 for f in floats if f != 0.0)
            all_bytes = sum(resp)
            print(f"Read {i+1}: Non-zero floats: {non_zero}/24, Byte sum: {all_bytes}")
            if non_zero > 0:
                print(f"  First 8 floats: {floats[:8]}")
            time.sleep(0.01)
            
        spi.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    print("SPI Slow Speed Test (1MHz)")
    print("=" * 40)
    
    test_spi(0, "Pico 2 A (Sensors)", 1000000)
    test_spi(1, "Pico 2 B (Actuators)", 1000000)
    
    print("\n=== Test Complete ===")
