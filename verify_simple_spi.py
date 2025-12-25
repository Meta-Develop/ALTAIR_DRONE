import spidev
import time
import RPi.GPIO as GPIO

# SPI Config for SPI0
BUS = 0
DEVICE = 0
CS_PIN = 25 # GPIO 25 (Sensor Bus / Standard SPI0 CS)

GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH)

spi = spidev.SpiDev()
spi.open(BUS, DEVICE)
spi.max_speed_hz = 100000
spi.mode = 0
spi.no_cs = True # We toggle CS manually

def test():
    print(f"Reading SPI0 (Manual CS={CS_PIN})... Expecting AA BB CC DD")
    
    # Send 4 bytes of junk to clock out data
    tx_buf = [0x00] * 4
    
    GPIO.output(CS_PIN, GPIO.LOW)
    rx_buf = spi.xfer2(tx_buf)
    GPIO.output(CS_PIN, GPIO.HIGH)
    
    bytes_rx = bytearray(rx_buf)
    print(f"RX: {bytes_rx.hex().upper()}")
    
    if bytes_rx == b'\xAA\xBB\xCC\xDD':
        print("SUCCESS! Received Simple Pattern.")
    elif bytes_rx == b'\x00\x00\x00\x00':
        print("FAILURE. Received Zeros.")
    else:
        print("FAILURE. Unknown Data.")

if __name__ == "__main__":
    try:
        while True:
            test()
            time.sleep(0.5)
    except KeyboardInterrupt:
        spi.close()
        GPIO.cleanup()
