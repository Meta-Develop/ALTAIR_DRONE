import spidev
import time
import RPi.GPIO as GPIO

# SPI Config for SPI1 (Actuator Bus) but testing Sensor Protocol
BUS = 1
DEVICE = 0 # spidev1.0
CS_PIN = 16 # Manual CS

GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)

spi = spidev.SpiDev()
spi.open(BUS, DEVICE)
spi.max_speed_hz = 1000000
spi.mode = 0
spi.no_cs = False # Allow kernel CS0 (GPIO 18) to fire harmlessly

def test():
    print(f"Checking SPI1 for SENSOR FW (Manual CS={CS_PIN})...")
    
    tx_buf = [0] * 128
    
    GPIO.output(CS_PIN, GPIO.LOW)
    rx_buf = spi.xfer2(tx_buf)
    GPIO.output(CS_PIN, GPIO.HIGH)
    
    bytes_rx = bytearray(rx_buf)
    print(f"RX: {bytes_rx[:16].hex()}...")
    
    if bytes_rx[0] == 0xAA and bytes_rx[1] == 0x55:
        print("SUCCESS! Found SENSOR FW on SPI1.")
    elif bytes_rx[0] == 0xCA and bytes_rx[1] == 0xFE:
        print("SURPRISE! Found ACTUATOR FW on SPI1.")
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
