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
        # Scan for Magic 0x55AA (AA 55 in LE) to handle alignment shifts
        offset = -1
        for i in range(0, len(rx)-1):
            if rx[i] == 0xAA and rx[i+1] == 0x55:
                offset = i
                break
        
        if offset >= 0:
            magic = struct.unpack_from('<H', rx, offset)[0]
            if offset + 13 < len(rx):
                frame_id = struct.unpack_from('<H', rx, offset+2)[0]
                timestamp = struct.unpack_from('<Q', rx, offset+4)[0]
                count = rx[offset+12]
                
                print(f"Header: OK (Offset {offset}). ID={frame_id}, Time={timestamp/1e6:.3f}s, Count={count}")
                if count > 0:
                    print(f"  SUCCESS: Received {count} samples!")
                    
                    # Decode Samples if requested
                    # sample_start = offset + 13
                    # ... decoding logic ...
                else:
                    print("  WARNING: Count=0. Sensors idle.")
            else:
                 print(f"  Found Magic at {offset} but packet truncated.")
        else:
            magic_le = struct.unpack_from('<H', rx, 0)[0]
            print(f"  FAILURE: Invalid Magic {hex(magic_le)} (Expected 0x55aa) - No Magic found.")
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
