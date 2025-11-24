#!/bin/bash
# ALTAIR Drone - Node C Optimization Script
# Target: Raspberry Pi 4 (Ubuntu 22.04)

echo "Applying OS Optimizations for Real-Time Control..."

# 1. Set CPU Governor to Performance
if [ -d "/sys/devices/system/cpu/cpu0/cpufreq" ]; then
    echo "Setting CPU governor to performance..."
    echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
else
    echo "CPU frequency scaling not found or accessible."
fi

# 2. IRQ Balancing (Optional: Move IRQs away from isolated core)
# Assuming Core 3 is isolated for NMPC solver
# Edit /etc/default/irqbalance to exclude core 3

# 3. Real-Time Throttling
# Disable RT throttling to allow 100% utilization by RT tasks
echo -1 | sudo tee /proc/sys/kernel/sched_rt_runtime_us

echo "Optimization applied."
echo "To fully isolate Core 3, add 'isolcpus=3' to /boot/cmdline.txt and reboot."
echo "Run the controller with: taskset -c 3 ros2 launch altair_controller controller.launch.py"
