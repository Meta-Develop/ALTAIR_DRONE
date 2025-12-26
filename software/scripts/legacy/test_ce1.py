#!/usr/bin/env python3
"""Test CE1 to check for Swapped Picos"""
import spidev
import time

def test_ce1():
    print("Testing CE1 (Pico 2 B?) for 10 seconds...")
    print("WATCH PICO 2 A (Sensors) LED!")
    
    spi = spidev.SpiDev()
    spi.open(0, 1) # CE1
    spi.max_speed_hz = 100000
    
    for i in range(20): # 10s (0.5s * 20)
        # Send burst
        spi.xfer2([0xAA]*10)
        time.sleep(0.5)
        print(f".", end="", flush=True)
        
    print("\nDone.")

if __name__ == "__main__":
    test_ce1()
