import time
import struct
import sys
import os


# GPIO 5 = Pin 29
PIN = 5

def main():
    print(f"Toggling GPIO {PIN} (Pin 29) every 1 second.")
    print("If wired to Pico Pin 22 (CS), the Pico might react if firmware is running.")
    print("Or you can measure voltage on Pin 29.")
    
    try:
        with open('/dev/gpiomem', 'r+b') as f:
            mem = mmap.mmap(f.fileno(), 4096)
    except Exception as e:
        print(f"Error opening /dev/gpiomem: {e}")
        return

    # FSEL
    fsel_offset = (PIN // 10) * 4
    shift = (PIN % 10) * 3
    
    mem.seek(fsel_offset)
    reg = struct.unpack('<I', mem.read(4))[0]
    reg &= ~(0b111 << shift)
    reg |= (0b001 << shift) # Output
    mem.seek(fsel_offset)
    mem.write(struct.pack('<I', reg))
    
    try:
        while True:
            # Low
            print("LOW")
            mem.seek(0x28) # GPCLR0
            mem.write(struct.pack('<I', 1 << PIN))
            time.sleep(1.0)
            
            # High
            print("HIGH")
            mem.seek(0x1C) # GPSET0
            mem.write(struct.pack('<I', 1 << PIN))
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nExiting")
        mem.close()

if __name__ == "__main__":
    main()
