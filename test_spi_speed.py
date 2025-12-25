#!/usr/bin/env python3
import spidev
import RPi.GPIO as GPIO
import time

CS_PIN = 25
SPI_SPEED = 8000000

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.output(CS_PIN, 1)

s = spidev.SpiDev()
s.open(0, 0)
s.mode = 0
s.no_cs = True

for CS_DELAY in [0.00005, 0.0001, 0.0002, 0.0005]:  # 50us, 100us, 200us, 500us
    s.max_speed_hz = 1_000_000  # Use 1MHz
    successes = 0
    for i in range(100):
        GPIO.output(CS_PIN, 0)
        time.sleep(CS_DELAY)
        rx = s.xfer2([0]*128)
        GPIO.output(CS_PIN, 1)
        
        # Check if magic is in first 4 bytes
        if 0xAA in rx[:4] or 0x55 in rx[:4]:
            successes += 1
        time.sleep(0.001)  # 1ms between reads
    print(f"CS Delay {CS_DELAY*1e6:.0f}us @ 1MHz: {successes}/100")

s.close()
GPIO.cleanup()
