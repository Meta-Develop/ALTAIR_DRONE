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
spi.max_speed_hz = 10000  # Lowered to 10kHz for debugging SCK signal integrity
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
        magic = struct.unpack_from('<H', rx, 0)[0]
        frame_id = struct.unpack_from('<H', rx, 2)[0]
        timestamp = struct.unpack_from('<Q', rx, 4)[0]
        count = rx[12]
        
        # 0xAA 0x55 in LE is 0x55AA
        if magic == 0x55AA:
            print(f"Header: OK. ID={frame_id}, Time={timestamp/1e6:.3f}s, Count={count}")
    
            if count > 0:
                # Decode Samples
                # Each sample: 6x int16 (Gyro X,Y,Z, Accel X,Y,Z) = 12 bytes
                offset = 16
                for i in range(count):
                    if offset + 12 > len(rx):
                        break
                    
                    # Assume [GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ]
                    data = struct.unpack_from('<6h', rx, offset)
                    gx, gy, gz, ax, ay, az = data
                    
                    print(f"  Sample {i}: G=({gx},{gy},{gz}) A=({ax},{ay},{az})")
                    offset += 12
            else:
                print("  WARNING: Count=0. Sensors idle.")
                
        else:
            print(f"  FAILURE: Invalid Magic {hex(magic)} (Expected 0x55aa)")
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
