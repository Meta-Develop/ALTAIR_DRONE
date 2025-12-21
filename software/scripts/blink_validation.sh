#!/bin/bash
# Blink Validation
# Pin 19 (GPIO 10) - SPI MOSI (Suspect Dead)
# Pin 37 (GPIO 26) - Unused GPIO (Control Good)

echo "Exporting GPIO 10 and 26"
echo 10 > /sys/class/gpio/export || echo "10 already exported"
echo 26 > /sys/class/gpio/export || echo "26 already exported"
sleep 1

echo "Setting Direction OUT"
echo out > /sys/class/gpio/gpio10/direction
echo out > /sys/class/gpio/gpio26/direction

echo "Blinking BOTH Pin 19 and Pin 37... (Ctrl+C to stop)"
while true; do
    echo 1 > /sys/class/gpio/gpio10/value
    echo 1 > /sys/class/gpio/gpio26/value
    sleep 0.5
    echo 0 > /sys/class/gpio/gpio10/value
    echo 0 > /sys/class/gpio/gpio26/value
    sleep 0.5
done
