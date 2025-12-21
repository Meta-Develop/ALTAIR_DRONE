import time
import os
import sys

# RPi4 GPIO Numbers (BCM)
SCLK_PIN = 11  # SPI0 SCLK usually Pin 23 is BCM 11. Wait.
# Pin 23 (Phys) = BCM 11 (SCLK). 
# But loopback test used Pin 19 (MOSI/BCM10) and Pin 21 (MISO/BCM9).
# SCLK is Pin 23 -> BCM 11.

def export_pin(pin, direction="out"):
    path = f"/sys/class/gpio/gpio{pin}"
    if not os.path.exists(path):
        with open("/sys/class/gpio/export", "w") as f:
            f.write(str(pin))
    time.sleep(0.1)
    with open(os.path.join(path, "direction"), "w") as f:
        f.write(direction)

def set_pin(pin, value):
    with open(f"/sys/class/gpio/gpio{pin}/value", "w") as f:
        f.write("1" if value else "0")

def cleanup(pin):
    try:
        with open("/sys/class/gpio/unexport", "w") as f:
            f.write(str(pin))
    except:
        pass

print("Starting SCLK Manual Toggle Test...")
print("GPIO 11 (SCLK) -> Pico GP18")

try:
    cleanup(SCLK_PIN)
    export_pin(SCLK_PIN, "out")
    
    print("Toggle Loop Started (2 sec period)")
    print("watch the Pico LED...")
    while True:
        print("SCLK HIGH -> LED ON?")
        set_pin(SCLK_PIN, 1)
        time.sleep(1.0)
        
        print("SCLK LOW  -> LED OFF?")
        set_pin(SCLK_PIN, 0)
        time.sleep(1.0)

except KeyboardInterrupt:
    print("\nAborted.")
finally:
    cleanup(SCLK_PIN)
