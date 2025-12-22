#!/bin/bash
GPIO=9
if [ ! -d /sys/class/gpio/gpio$GPIO ]; then
    echo "Exporting GPIO $GPIO"
    echo $GPIO > /sys/class/gpio/export
    sleep 1
fi
echo in > /sys/class/gpio/gpio$GPIO/direction

echo "Monitoring GPIO $GPIO (MISO) Input. Press Ctrl+C to stop."
while true; do
  cat /sys/class/gpio/gpio$GPIO/value
  sleep 0.1
done
