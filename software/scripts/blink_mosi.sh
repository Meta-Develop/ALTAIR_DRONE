#!/bin/bash
# Manually blink GPIO 10 (MOSI) using sysfs
# This verifies if the electrical pad is working, bypassing SPI controller.

GPIO=10

echo "Exporting GPIO $GPIO"
echo $GPIO > /sys/class/gpio/export || echo "Already exported"
sleep 1

echo "Setting Direction OUT"
echo out > /sys/class/gpio/gpio$GPIO/direction

echo "Blinking... (Ctrl+C to stop)"
while true; do
    echo 1 > /sys/class/gpio/gpio$GPIO/value
    sleep 0.5
    echo 0 > /sys/class/gpio/gpio$GPIO/value
    sleep 0.5
done
