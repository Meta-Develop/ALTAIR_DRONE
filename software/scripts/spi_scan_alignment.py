import spidev
import struct
import time

def shift_buf(buf, shift):
    # Left shift buffer by 'shift' bits
    # Returns a new bytearray
    out = bytearray(len(buf))
    carry = 0
    for i in range(len(buf)-1, -1, -1):
        val = buf[i]
        new_val = ((val << shift) | carry) & 0xFF
        carry = (val >> (8 - shift))
        out[i] = new_val
    return out

def test_alignment(ce):
    print(f"--- Scanning CE{ce} ---")
    spi = spidev.SpiDev()
    spi.open(0, ce) # CE0 or CE1
    spi.max_speed_hz = 100000 # 100kHz
    spi.mode = 0
    
    # Send 0xFF to drive MOSI High (allows diagnosing MISO Loopback if needed)
    # But strictly we just want to read MISO.
    msg = [0xFF] * 100
    raw = spi.xfer2(msg)
    raw_bytes = bytearray(raw)
    
    print(f"Raw Head: {[hex(x) for x in raw[:10]]}")
    
    # Check for silence
    if all(x == 0 for x in raw):
        print(f"[FAIL] CE{ce}: All Zeros (No Response)")
        spi.close()
        return

    # Try Shifts
    found = False
    for shift in range(8):
        shifted = shift_buf(raw_bytes, shift)
        # Unpack as floats
        try:
            # We look for 0xAA 0xBB 0xCC 0xDD pattern at start?
            # Or just any valid float data
            # Check for Debug Header 0xAA 0xBB 0xCC 0xDD
            # It might appear anywhere due to sync loss
            
            # Simple check: DO WE SEE 0xAA?
            if 0xAA in shifted:
                 print(f"[POTENTIAL] Shift {shift}: Found 0xAA!")
                 found = True
                 
        except Exception as e:
            pass
            
    spi.close()

if __name__ == "__main__":
    test_alignment(0) # Sensors
    test_alignment(1) # Actuators
