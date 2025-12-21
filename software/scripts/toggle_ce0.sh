#!/bin/bash
# Toggle GPIO 8 (CE0) manually using sysfs

GPIO=8
SYS_GPIO=/sys/class/gpio

echo "Setting up GPIO $GPIO (CE0)..."

# Export if not already exported
if [ ! -d "$SYS_GPIO/gpio$GPIO" ]; then
    echo "$GPIO" > "$SYS_GPIO/export"
    sleep 0.1
fi

# Set direction to out
echo "out" > "$SYS_GPIO/gpio$GPIO/direction"

echo "Toggling CE0 at 1Hz... Watch Pico 2 A LED!"
echo "Press Ctrl+C to stop"

while true; do
    echo "1" > "$SYS_GPIO/gpio$GPIO/value"
    echo "HIGH (LED OFF?)" # Pico Diag toggles on EDGE, so any change blinks it
    sleep 0.5
    
    echo "0" > "$SYS_GPIO/gpio$GPIO/value"
    echo "LOW  (LED BLINK?)"
    sleep 0.5
done
