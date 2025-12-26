import spidev
import time
import struct

# Config
# SPI Bus 0, Device 0 (CE0)
BUS = 0
DEVICE = 0
SPEED_HZ = 100000 # 100kHz (Lowered for debugging)

def main():
    spi = spidev.SpiDev()
    try:
        spi.open(BUS, DEVICE)
        spi.max_speed_hz = SPEED_HZ
        spi.mode = 0 # CPOL=0, CPHA=0
        
        print(f"SPI Initialized: Bus {BUS}, Device {DEVICE}, Speed {SPEED_HZ}Hz")
        
        # Payload size: 8 floats * 4 bytes = 32 bytes
        PACKET_SIZE = 32
        dummy_tx = [0] * PACKET_SIZE
        
        last_print = time.time()
        count = 0
        
        while True:
            # Transfer 32 bytes
            # xfer2 keeps CS asserted for the whole transaction
            rx_raw = spi.xfer2(dummy_tx)
            
            # Interpret
            if len(rx_raw) == PACKET_SIZE:
                try:
                    # '8f' = 8 floats (little endian standard)
                    data = struct.unpack('<8f', bytearray(rx_raw))
                    
                    sec = data[0]
                    nsec = data[1]
                    ax, ay, az = data[2:5]
                    gx, gy, gz = data[5:8]
                    
                    count += 1
                    if time.time() - last_print > 0.5:
                        hz = count / 0.5
                        print(f"[{hz:.1f} Hz] T={sec}.{int(nsec/1000):06d} | A=({ax:.2f}, {ay:.2f}, {az:.2f}) G=({gx:.2f}, {gy:.2f}, {gz:.2f})")
                        count = 0
                        last_print = time.time()
                        
                except Exception as e:
                    print(f"Parse Error: {e}")
            
            # Target 1kHz (sleep 1ms)
            # time.sleep(0.001) 
            # Note: For max speed test, remove sleep.
            time.sleep(0.0005)

    except Exception as e:
        print(f"SPI Error: {e}")
    finally:
        spi.close()

if __name__ == "__main__":
    main()
