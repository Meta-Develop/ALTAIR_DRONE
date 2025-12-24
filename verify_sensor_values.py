import spidev
import time
import RPi.GPIO as GPIO
import struct

# SPI Config for Pico 2A (Sensors)
BUS = 0
DEVICE = 0
CS_PIN = 25 # GPIO 25 (Pin 22) matches Pico 2A Wiring

GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)

spi = spidev.SpiDev()
spi.open(BUS, DEVICE)
spi.max_speed_hz = 1000000
spi.mode = 0
spi.no_cs = True 

def test_sensor():
    print(f"Reading Sensor Data (SPI0 CS={CS_PIN})...")
    
    # Send empty buffer to clock out data
    tx_buf = [0] * 128
    
    GPIO.output(CS_PIN, GPIO.LOW)
    rx_buf = spi.xfer2(tx_buf)
    GPIO.output(CS_PIN, GPIO.HIGH)
    
    rx = bytearray(rx_buf)
    
    try:
        # Check standard Magic (AA 55 in LE -> 0x55AA)
        offset_std = -1
        offset_swap = -1
        
        for i in range(0, len(rx)-3):
            # Standard: AA 55
            if rx[i] == 0xAA and rx[i+1] == 0x55:
                offset_std = i
                break
            # Swapped: 55 AA (Word Swap artifact)
            if rx[i] == 0x55 and rx[i+1] == 0xAA:
                offset_swap = i
                break
                
        if offset_std >= 0:
            print(f"  Types: Standard Order found at Offset {offset_std}")
            base = offset_std
            # Decode Standard
            magic = struct.unpack_from('<H', rx, base)[0]
            if base + 13 < len(rx):
                frame_id = struct.unpack_from('<H', rx, base+2)[0]
                timestamp = struct.unpack_from('<Q', rx, base+4)[0]
                count = rx[base+12]
                print(f"  Header: OK. ID={frame_id}, Time={timestamp/1e6:.3f}s, Count={count}")
                if count > 0: print(f"  SUCCESS: Received {count} samples!")
                else: print("  WARNING: Count=0")

        elif offset_swap >= 0:
            print(f"  Type: Swapped Order found at Offset {offset_swap}")
            # Unswap 16-bit words: [A, B] -> [B, A]
            # Actually, if we see 55 AA, it means buffer has 55 AA instead of AA 55.
            # We can re-pack correctly.
            # Only need to unswap the region we care about.
            valid_len = len(rx) - offset_swap
            unswapped = bytearray(valid_len)
            for j in range(0, valid_len-1, 2):
                unswapped[j] = rx[offset_swap+j+1]
                unswapped[j+1] = rx[offset_swap+j]
                
            # Now decode standard from unswapped[0]
            frame_id = struct.unpack_from('<H', unswapped, 2)[0]
            timestamp = struct.unpack_from('<Q', unswapped, 4)[0]
            count = unswapped[12]
            
            print(f"  Header: OK (Swapped). ID={frame_id}, Time={timestamp/1e6:.3f}s, Count={count}")
            if count > 0: print(f"  SUCCESS: Received {count} samples!")
            else: print("  WARNING: Count=0")
            
        else:
            print(f"  FAILURE: No valid magic found.")
            print(f"  Raw: {rx[:16].hex()}...")

    except Exception as e:
        print(f"  Decode Error: {e}")

if __name__ == "__main__":
    try:
        while True:
            test_sensor()
            time.sleep(0.5)
    except KeyboardInterrupt:
        spi.close()
        GPIO.cleanup()
