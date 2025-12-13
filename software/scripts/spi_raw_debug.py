import spidev
import time
import struct

BUS = 0
DEVICE = 0
SPEED_HZ = 100000 

def main():
    spi = spidev.SpiDev()
    try:
        spi.open(BUS, DEVICE)
        spi.max_speed_hz = SPEED_HZ
        spi.mode = 0
        
        print(f"SPI Raw Debug: Bus {BUS}, Device {DEVICE}, Speed {SPEED_HZ}Hz")
        
        PACKET_SIZE = 32
        dummy_tx = [0] * PACKET_SIZE
        
        while True:
            rx_raw = spi.xfer2(dummy_tx)
            # Print raw bytes to see pattern
            print(f"RX: {rx_raw}")
            time.sleep(0.1) # Slow down to read

    except Exception as e:
        print(f"Error: {e}")
    finally:
        spi.close()

if __name__ == "__main__":
    main()
