import time
import os
import sys

# RPi4 MISO is Physical Pin 21 -> BCM 9
MISO_PIN = 9

def export_pin(pin):
    path = f"/sys/class/gpio/gpio{pin}"
    if not os.path.exists(path):
        with open("/sys/class/gpio/export", "w") as f:
            f.write(str(pin))
    time.sleep(0.1)
    with open(os.path.join(path, "direction"), "w") as f:
        f.write("in")

def get_pin(pin):
    with open(f"/sys/class/gpio/gpio{pin}/value", "r") as f:
        return int(f.read().strip())

def cleanup(pin):
    try:
        with open("/sys/class/gpio/unexport", "w") as f:
            f.write(str(pin))
    except:
        pass

print("Starting MISO Monitor...")
print("Reading GPIO 9 (MISO)... Expecting 1Hz Toggle.")

try:
    cleanup(MISO_PIN)
    export_pin(MISO_PIN)
    
    while True:
        val = get_pin(MISO_PIN)
        bar = "#" if val else "_"
        print(f"{bar}", end="", flush=True)
        time.sleep(0.05) # 20Hz sample rate

except KeyboardInterrupt:
    print("\nAborted.")
finally:
    cleanup(MISO_PIN)
    print("\nClosed.")
