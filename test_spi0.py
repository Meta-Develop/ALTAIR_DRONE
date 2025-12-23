import spidev
import time
import RPi.GPIO as GPIO

# SPI Config for SPI0 (Sensor Bus)
BUS = 0
DEVICE = 0
# GPIO 22 is CS0. WIRING_DIAGRAM says CS is GPIO 25 (Pin 22).
# Let's try Manual CS on GPIO 25 first as per diagram.
CS_PIN = 25 

GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)

spi = spidev.SpiDev()
spi.open(BUS, DEVICE)
spi.max_speed_hz = 100000 
spi.mode = 0
spi.no_cs = True 

def calculate_checksum(data):
    xor = 0
    for b in data:
        xor ^= b
    return xor

def test():
    print(f"Testing Pico (SPI0) Manual CS={CS_PIN}...")
    cmd = bytearray(16)
    cmd[0] = 0xBA
    cmd[1] = 0xBE
    cmd[15] = calculate_checksum(cmd[:15])
    tx_buf = list(cmd + bytearray(30))
    
    GPIO.output(CS_PIN, GPIO.LOW)
    rx_buf = spi.xfer2(tx_buf)
    GPIO.output(CS_PIN, GPIO.HIGH)
    
    bytes_rx = bytearray(rx_buf)
    
    print(f"TX: {bytes(tx_buf[:16]).hex()}...")
    print(f"RX: {bytes_rx.hex()}")
    
    if bytes_rx[0] == 0xCA and bytes_rx[1] == 0xFE:
        print("SUCCESS! Found Actuator FW on SPI0 (Sensor Bus).")
    else:
        print("FAILURE. No response on SPI0.")

if __name__ == "__main__":
    try:
        test()
    except KeyboardInterrupt:
        pass
    finally:
        spi.close()
        GPIO.cleanup()
