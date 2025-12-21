#!/bin/bash
# Robust Pico Flasher
# Finds the device labeled RP2350 and flashes the given UF2 file

UF2_FILE=$1

if [ -z "$UF2_FILE" ]; then
    echo "Usage: $0 <path_to_uf2>"
    exit 1
fi

echo "Searching for Pico 2 (RP2350)..."
DEV=$(lsblk -rn -o NAME,LABEL | grep RP2350 | awk '{print $1}')

if [ -z "$DEV" ]; then
    echo "Error: No RP2350 device found in BOOTSEL mode."
    lsblk
    exit 1
fi

echo "Found device: /dev/$DEV"

echo "Mounting..."
sudo umount /mnt/pico_flash 2>/dev/null
sudo mkdir -p /mnt/pico_flash
sudo mount /dev/$DEV /mnt/pico_flash

if [ $? -ne 0 ]; then
    echo "Error: Failed to mount /dev/$DEV"
    exit 1
fi

echo "Flashing $UF2_FILE..."
sudo cp "$UF2_FILE" /mnt/pico_flash/
sync

echo "Flash Success!"
