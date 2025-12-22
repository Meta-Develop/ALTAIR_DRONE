#!/bin/bash
# Set GPIO 10 (Pin 19) and GPIO 26 (Pin 37) to HIGH
echo "Exporting GPIO 10 and 26..."
echo 10 > /sys/class/gpio/export 2>/dev/null
echo 26 > /sys/class/gpio/export 2>/dev/null
sleep 0.5

echo "Setting Direction OUT..."
echo out > /sys/class/gpio/gpio10/direction
echo out > /sys/class/gpio/gpio26/direction

echo "Setting HIGH..."
echo 1 > /sys/class/gpio/gpio10/value
echo 1 > /sys/class/gpio/gpio26/value

echo "Done. Pins 19 and 37 should be 3.3V."
