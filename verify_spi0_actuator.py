import spidev
import time
import RPi.GPIO as GPIO
import struct

# SPI Config for SPI0 (Sensor Bus) testing Actuator Pico
BUS = 0
DEVICE = 0 
CS_PIN = 25 # GPIO 25 (Pin 22) - Standard SPI0 CS

GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)

spi = spidev.SpiDev()
spi.open(BUS, DEVICE)
spi.max_speed_hz = 100000
spi.mode = 0
spi.no_cs = True # Manual CS

def calculate_checksum(data):
    xor = 0
    for b in data:
        xor ^= b
    return xor

def test():
    print(f"Checking SPI0 for ACTUATOR FW (Manual CS={CS_PIN})...")
    
    cmd = bytearray(16)
    cmd[0] = 0xBA
    cmd[1] = 0xBE
    cmd[15] = calculate_checksum(cmd[:15])
    
    # 128 bytes total (Actuator Packet size + padding)
    tx_buf = list(cmd + bytearray(112))
    
    GPIO.output(CS_PIN, GPIO.LOW)
    rx_buf = spi.xfer2(tx_buf)
    GPIO.output(CS_PIN, GPIO.HIGH)
    
    bytes_rx = bytearray(rx_buf)
    print(f"RX Header: {bytes_rx[:16].hex()}...")
    
    if bytes_rx[0] == 0xCA and bytes_rx[1] == 0xFE:
        print("SUCCESS! Found ACTUATOR FW on SPI0.")
        # Decode Telemetry (ActuatorPacket)
        # 0: Magic(2), 2: Flags(1), 3: Extra(1), 4: MotorRPM(12), 8: Voltage(2), ...
        # Based on firmware struct:
        # u16 magic
        # u8  flags, internal_state
        # s16 rpm[6]
        # u16 voltage
        # u16 current
        # u16 temperature
        # u8  rx_throttle[6]
        # u8  checksum
        
        try:
            # Skip valid magic check since we did it above, unpack rest
            # < = little endian, H = u16, B = u8, h = s16
            magic, flags, state = struct.unpack_from('<HBB', bytes_rx, 0)
            rpms = struct.unpack_from('<6h', bytes_rx, 4)
            voltage, current, temp = struct.unpack_from('<HHH', bytes_rx, 16)
            
            print(f"  Telemetry: RPM={rpms}, V={voltage/100.0}V, I={current/100.0}A, T={temp/100.0}C")
        except Exception as e:
            print(f"  Decoding Error: {e}")
            
    elif bytes_rx[0] == 0xAA and bytes_rx[1] == 0x55:
         print("NOTE: Still seeing SENSOR FW on SPI0 (Swap not active?).")
    else:
        print("FAILURE. No valid magic bytes.")

if __name__ == "__main__":
    try:
        while True:
            test()
            time.sleep(1.0)
    except KeyboardInterrupt:
        spi.close()
        GPIO.cleanup()
