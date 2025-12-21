#!/bin/bash
# Export and Configure GPIOs for Manual CS
# Run with sudo

# GPIO 5 (Pin 29) - Sensors CS
if [ ! -d /sys/class/gpio/gpio5 ]; then
    echo 5 > /sys/class/gpio/export
fi
echo out > /sys/class/gpio/gpio5/direction
chmod 666 /sys/class/gpio/gpio5/value
echo "GPIO 5 Configured"

# GPIO 16 (Pin 36) - Actuators CS
if [ ! -d /sys/class/gpio/gpio16 ]; then
    echo 16 > /sys/class/gpio/export
fi
echo out > /sys/class/gpio/gpio16/direction
chmod 666 /sys/class/gpio/gpio16/value
echo "GPIO 16 Configured"

# GPIO 6 (Pin 31) - Data Ready (Input)
if [ ! -d /sys/class/gpio/gpio6 ]; then
    echo 6 > /sys/class/gpio/export
fi
echo in > /sys/class/gpio/gpio6/direction
echo rising > /sys/class/gpio/gpio6/edge
chmod 666 /sys/class/gpio/gpio6/value
chmod 666 /sys/class/gpio/gpio6/edge
echo "GPIO 6 Configured"

# GPIO 9 (Pin 21) - MISO Monitor
if [ ! -d /sys/class/gpio/gpio9 ]; then
    echo 9 > /sys/class/gpio/export
fi
echo in > /sys/class/gpio/gpio9/direction
chmod 666 /sys/class/gpio/gpio9/value
echo "GPIO 9 Configured"
