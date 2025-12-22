#!/bin/bash
GPIO=5
if [ ! -d /sys/class/gpio/gpio$GPIO ]; then
    echo "Exporting GPIO $GPIO"
    echo $GPIO > /sys/class/gpio/export
    sleep 1
fi
echo out > /sys/class/gpio/gpio$GPIO/direction

echo "Toggling GPIO $GPIO (CS Line) forever. Press Ctrl+C to stop."
echo "1. Measure voltage at RPi4 Pin 29 (Should toggle 0V / 3.3V)"
echo "2. Measure voltage at Pico Pin 22 (Should toggle 0V / 3.3V)"

while true; do
  echo 1 > /sys/class/gpio/gpio$GPIO/value
  # echo "HIGH"
  sleep 0.5
  echo 0 > /sys/class/gpio/gpio$GPIO/value
  # echo "LOW"
  sleep 0.5
done
