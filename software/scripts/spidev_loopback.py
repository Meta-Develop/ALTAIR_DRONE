import spidev
import time
import os

def test_spi_loopback():
    spi = spidev.SpiDev()
    for mode in [0, 1, 2, 3]:
        print(f"\nTesting SPI Mode {mode}...")
        try:
            spi.open(0, 0)
            spi.max_speed_hz = 50000
            spi.mode = mode
            
            test_data = [0x01, 0x02, 0x03, 0x42, 0xAA, 0xFF]
            rx_data = spi.xfer2(list(test_data))
            print(f"Received: {[hex(x) for x in rx_data]}")
            
            if 0xAA in rx_data or 0x2A in rx_data:
                 print(f"Data Detected in Mode {mode}!")
            
            spi.close()
        except Exception as e:
            print(f"Mode {mode} Error: {e}")
            
    return True

if __name__ == "__main__":
    test_spi_loopback()
