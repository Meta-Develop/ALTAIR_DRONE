#!/bin/bash

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

CMDLINE_FILE="/boot/firmware/cmdline.txt"

# Backup
cp $CMDLINE_FILE "${CMDLINE_FILE}.bak"

# Check if isolcpus is already set
if grep -q "isolcpus=2,3" "$CMDLINE_FILE"; then
    echo "CPU isolation already configured."
else
    echo "Configuring CPU isolation for cores 2 and 3..."
    # Append to the end of the line, ensuring space
    sed -i 's/$/ isolcpus=2,3/' "$CMDLINE_FILE"
    echo "Done. Please reboot for changes to take effect."
fi

# Verify
cat $CMDLINE_FILE
