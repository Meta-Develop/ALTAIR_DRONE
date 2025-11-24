#!/bin/bash

# System Optimization Script for ALTAIR Drone Node B
# Requires sudo privileges

echo "Applying System Optimizations for Real-Time ROS 2 Performance..."

# 1. Dedicate CPUs 2 and 3 to ROS processes (Kernel boot arg setup)
# Note: This script modifies grub. verification required by user.
# Ideally, we append 'isolcpus=2,3' to GRUB_CMDLINE_LINUX_DEFAULT in /etc/default/grub
if grep -q "isolcpus=2,3" /etc/default/grub; then
    echo "isolcpus already configured."
else
    echo "Warning: 'isolcpus=2,3' not found in /etc/default/grub."
    echo "To apply manually: Add 'isolcpus=2,3' to GRUB_CMDLINE_LINUX_DEFAULT and run update-grub."
    # We do not automatically modify grub to avoid breaking the system without explicit confirmation.
fi

# 2. Set USB Latency Timer to 1ms for U2D2 (FTDI devices)
# U2D2 usually shows up as ttyUSBx. We'll try to set it for all ttyUSB devices found.
echo "Setting USB Latency to 1ms for ttyUSB devices..."

for dev in /sys/bus/usb-serial/devices/ttyUSB*; do
    if [ -d "$dev" ]; then
        echo 1 | sudo tee "$dev/latency_timer" > /dev/null
        if [ $? -eq 0 ]; then
            echo "Successfully set latency for $(basename $dev)"
        else
            echo "Failed to set latency for $(basename $dev)"
        fi
    fi
done

# 3. Set CPU scaling governor to performance
echo "Setting CPU governor to performance..."
if command -v cpufreq-set &> /dev/null; then
    sudo cpufreq-set -r -g performance
    echo "CPU governor set to performance."
else
    echo "cpufreq-set not found. Installing cpufrequtils..."
    # sudo apt-get install -y cpufrequtils
    # sudo cpufreq-set -r -g performance
    echo "Please install 'cpufrequtils' and run 'sudo cpufreq-set -r -g performance'"
fi

echo "Optimization script complete."
