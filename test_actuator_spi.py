import spidev
import time
import RPi.GPIO as GPIO

# SPI Config
BUS = 1
DEVICE = 2 # Placeholder, we use Manual CS
CS_PIN = 16 # GPIO 16 (Pin 36) - Actuator CS

# Setup Manual CS
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH) # Idle High

spi = spidev.SpiDev()
spi.open(BUS, DEVICE)
spi.max_speed_hz = 100000 
spi.mode = 0
spi.no_cs = True # Disable HW CS

def calculate_checksum(data):
    xor = 0
    for b in data:
        xor ^= b
    return xor

def test():
    print(f"Testing Pico (SPI1) Manual CS={CS_PIN}...")
    
    cmd = bytearray(16)
    cmd[0] = 0xBA
    cmd[1] = 0xBE
    cmd[15] = calculate_checksum(cmd[:15])
    
    tx_buf = list(cmd + bytearray(30))
    
    # Transaction
    GPIO.output(CS_PIN, GPIO.LOW)
    rx_buf = spi.xfer2(tx_buf)
    GPIO.output(CS_PIN, GPIO.HIGH)
    
    bytes_rx = bytearray(rx_buf)
    
    print(f"TX: {bytes(tx_buf[:16]).hex()}...")
    print(f"RX: {bytes_rx.hex()}")
    
    if bytes_rx[0] == 0xCA and bytes_rx[1] == 0xFE:
        print("SUCCESS! Received 0xCAFE Magic.")
    else:
        print("FAILURE. Magic header missing.")

if __name__ == "__main__":
    try:
        while True:
            test()
            time.sleep(0.5)
    except KeyboardInterrupt:
        spi.close()
        GPIO.cleanup()
