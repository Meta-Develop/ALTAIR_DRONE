#!/bin/bash
# Enable Low Latency mode for USB Serial (FTDI/CDC-ACM)
# Usage: ./set_low_latency.sh [device]
# Default: /dev/ttyACM0

DEV=${1:-/dev/ttyACM0}

echo "Configuring $DEV for low latency..."

# Check if setserial is installed
if ! command -v setserial &> /dev/null; then
    echo "Installing setserial..."
    sudo apt-get update && sudo apt-get install -y setserial
fi

# Set low_latency flag
if [ -c "$DEV" ]; then
    sudo setserial "$DEV" low_latency
    sudo setserial "$DEV"
    echo "Success: Low latency set for $DEV"
else
    echo "Error: Device $DEV not found!"
    exit 1
fi
