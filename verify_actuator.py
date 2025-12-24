import spidev
import time
import RPi.GPIO as GPIO
import struct

# SPI Config for Actuator Pico (SPI1)
BUS = 1
DEVICE = 0 
CS_PIN = 16 # GPIO 16 (Pin 36) - Actuator CS on SPI1

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)

# Setup SPI
spi = spidev.SpiDev()
try:
    spi.open(BUS, DEVICE)
    spi.max_speed_hz = 1000000 # 1MHz
    spi.mode = 0
    spi.no_cs = True # Manual CS
except Exception as e:
    print(f"SPI Open Failed (Bus {BUS}, Dev {DEVICE}): {e}")
    exit(1)

def calculate_checksum(data):
    xor = 0
    for b in data:
        xor ^= b
    return xor

def test():
    print(f"Checking Actuator Data (SPI{BUS} CS={CS_PIN})...")
    
    # Create Command Packet (16 bytes payload + padding)
    # Magic: 0xBABE (Command to Pico)
    cmd = bytearray(16)
    cmd[0] = 0xBA
    cmd[1] = 0xBE
    # ... rest zero ...
    cmd[15] = calculate_checksum(cmd[:15])
    
    # Full buffer with padding 
    # Actuator packet is small, but let's assume 32 bytes transfer for safety
    tx_buf = list(cmd + bytearray(16)) 
    
    GPIO.output(CS_PIN, GPIO.LOW)
    rx_buf = spi.xfer2(tx_buf)
    GPIO.output(CS_PIN, GPIO.HIGH)
    
    rx = bytearray(rx_buf)
    
    # Determine Checksum/Magic
    # Actuator FW sends 0xCAFE as Magic
    magic_std = struct.unpack_from('<H', rx, 0)[0] # Expect 0xFECA (LE of CAFE) -> Wait. CA FE in order.
    # 0xCA = 202, 0xFE = 254
    # LE 0xFECA = 65226
    
    print(f"RX Raw: {rx[:16].hex()}")
    
    if rx[0] == 0xCA and rx[1] == 0xFE:
        print("SUCCESS: Found 0xCAFE (Standard Order)")
        decode_telemetry(rx)
    elif rx[0] == 0xFE and rx[1] == 0xCA:
        print("SUCCESS: Found 0xCAFE (But Swapped/LE?) -> 0xFECA. This is unlikely for byte stream unless word swap.")
    elif rx[0] == 0x55 and rx[1] == 0xAA:
        print("FAILURE: Found 0x55AA (Sensor Magic). Wiring Swap Persists?!")
    else:
        # Scan for CAFE
        offset = -1
        for i in range(len(rx)-1):
            if rx[i] == 0xCA and rx[i+1] == 0xFE:
                offset = i
                break
        
        if offset >= 0:
             print(f"SUCCESS: Found 0xCAFE at Offset {offset}")
             decode_telemetry(rx[offset:])
        else:
             print("FAILURE: No valid Magic Byte.")

def decode_telemetry(data):
    try:
        # Struct: magic(2), flags(1), state(1), rpm(12), voltage(2), current(2), temp(2), rx_thr(6), ck(1)= Total 29 bytes
        if len(data) < 29:
             print("Packet too short to decode.")
             return
             
        magic, flags, state = struct.unpack_from('<HBB', data, 0)
        rpms = struct.unpack_from('<6h', data, 4)
        v, i, t = struct.unpack_from('<HHH', data, 16)
        
        print(f"  > RPM: {rpms}")
        print(f"  > Batt: {v/100.0:.2f}V, {i/100.0:.2f}A, {t/100.0:.2f}C")
        
    except Exception as e:
        print(f"Decode Error: {e}")

if __name__ == "__main__":
    try:
        while True:
            test()
            time.sleep(1.0)
    except KeyboardInterrupt:
        spi.close()
        GPIO.cleanup()
