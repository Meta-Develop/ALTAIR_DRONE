import spidev
import time
import RPi.GPIO as GPIO

# SPI Config for Pico 2A (Sensors)
BUS = 0
DEVICE = 0
CS_PIN = 25 # GPIO 25 (Pin 22) matches Pico 2A Wiring

# Setup Manual CS
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, GPIO.HIGH) # Idle High

spi = spidev.SpiDev()
spi.open(BUS, DEVICE)
spi.max_speed_hz = 1000000 # 1MHz
spi.mode = 0
spi.no_cs = True # Disable HW CS

def test_sensor():
    print(f"Verifying Pico 2A (Sensors) SPI0 Manual CS={CS_PIN}...")
    
    # Sensor Packet is 128 bytes
    # To trigger a read from the Pico Slave (TX Only PIO), 
    # we just need to provide clocks. xfer2 with 128 zeros.
    tx_buf = [0] * 128
    
    # Transaction
    GPIO.output(CS_PIN, GPIO.LOW) # CS Fall (Start)
    time.sleep(0.00001) # Small setup
    rx_buf = spi.xfer2(tx_buf)
    GPIO.output(CS_PIN, GPIO.HIGH) # CS Rise (End)
    
    bytes_rx = bytearray(rx_buf)
    
    print(f"RX (Partial): {bytes_rx[:16].hex()}...")
    
    if bytes_rx[0] == 0xAA and bytes_rx[1] == 0x55:
        print("SUCCESS! Received 0xAA55 Sensor Magic.")
        print(f"Frame ID: {bytes_rx[2]}")
        print(f"Sample Count: {bytes_rx[12]}")
    else:
        print("FAILURE. Sensor magic header missing.")

if __name__ == "__main__":
    try:
        while True:
            test_sensor()
            time.sleep(0.5)
    except KeyboardInterrupt:
        spi.close()
        GPIO.cleanup()
