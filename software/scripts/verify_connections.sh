#!/bin/bash

# verification_script.sh
# Automates the verification of ALTAIR Hardware Connections (Pico 2s, Sensors, Actuators)

# 1. Verification of Tools
if ! command -v openocd &> /dev/null; then
    echo "❌ OpenOCD not found. Please install it."
    exit 1
fi

echo "=========================================="
echo "   ALTAIR HARDWARE VERIFICATION STEPS"
echo "=========================================="

# 2. Flash Firmware (Ensure we are testing latest code)
echo ""
echo "--- [1/3] Flashing Firmware ---"

echo ">> Flashing Pico 2 A (Sensors)..."
# Try to flash Sensor Node
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000; program ~/ALTAIR_DRONE/software/firmware/pico_sensors/build/pico_sensors.elf verify reset exit"
if [ $? -eq 0 ]; then
    echo "✅ Sensor Node Flashed Successfully."
else
    echo "❌ Failed to Flash Sensor Node."
fi

echo ">> Flashing Pico 2 B (Actuators)..."
# Try to flash Actuator Node
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000; program ~/ALTAIR_DRONE/software/firmware/pico_actuators/build/pico_actuators.elf verify reset exit"
if [ $? -eq 0 ]; then
    echo "✅ Actuator Node Flashed Successfully."
else
    echo "❌ Failed to Flash Actuator Node."
fi

# 3. SPI Bridge Test
echo ""
echo "--- [2/3] Testing SPI Communication ---"
echo "Starting spi_bridge.py for 10 seconds..."

# Source ROS 2 (Adjust path if needed)
source /opt/ros/humble/setup.bash
source ~/ALTAIR_DRONE/software/RPi4_1_ws/install/setup.bash

# Run Bridge and capture output
ros2 run rpi4_1_bringup spi_bridge > spi_test.log 2>&1 &
BRIDGE_PID=$!

sleep 10
kill $BRIDGE_PID

echo ">> Checking SPI Bridge Logs:"
if grep -q "SPI Sensors (CE0) Initialized" spi_test.log; then
    echo "✅ SPI Bus Initialized."
else
    echo "❌ SPI Bus Failed Init."
fi

if grep -q "IMU Rate" spi_test.log; then
    echo "✅ IMU Data Detected!"
else
    echo "❌ No IMU Data Received (Check Sensor Wiring)."
fi

echo ""
echo "--- [3/3] RAW Log Output (Last 10 lines) ---"
tail -n 10 spi_test.log

echo ""
echo "✅ Verification Complete."
