#!/bin/bash
# cleanup_gpio.sh

echo "Killing spi_bridge_cpp..."
pkill -9 -f spi_bridge_cpp
sleep 1

echo "Unexporting GPIOs from sysfs..."
if [ -d /sys/class/gpio/gpio24 ]; then echo 24 > /sys/class/gpio/unexport; fi
if [ -d /sys/class/gpio/gpio25 ]; then echo 25 > /sys/class/gpio/unexport; fi

echo "Checking gpioinfo..."
gpioinfo | grep -E "lines 24|lines 25"

echo "Cleanup Complete."
