import RPi.GPIO as GPIO
import time

# Pin 36 = GPIO 16
CS_PIN = 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)

print(f"Toggling GPIO {CS_PIN} (Pin 36) every 0.1s...")

try:
    while True:
        GPIO.output(CS_PIN, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(CS_PIN, GPIO.HIGH)
        time.sleep(0.1)
except KeyboardInterrupt:
    GPIO.cleanup()
