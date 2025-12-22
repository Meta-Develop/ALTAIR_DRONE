#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# Pin Definitions (Board Mode)
PIN_MOSI = 19
PIN_CHECK = 37

print(f"Blinking Pin {PIN_MOSI} (MOSI) and Pin {PIN_CHECK} (Check)...")

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# Setup
GPIO.setup(PIN_MOSI, GPIO.OUT)
GPIO.setup(PIN_CHECK, GPIO.OUT)

try:
    while True:
        # ON
        GPIO.output(PIN_MOSI, GPIO.HIGH)
        GPIO.output(PIN_CHECK, GPIO.HIGH)
        time.sleep(0.5)
        
        # OFF
        GPIO.output(PIN_MOSI, GPIO.LOW)
        GPIO.output(PIN_CHECK, GPIO.LOW)
        time.sleep(0.5)
except KeyboardInterrupt:
    GPIO.cleanup()
