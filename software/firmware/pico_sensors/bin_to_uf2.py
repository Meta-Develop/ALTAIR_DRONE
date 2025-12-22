#!/usr/bin/env python3
"""
bin_to_uf2.py - Convert binary file to UF2 format for RP2350 (Pico 2)
Based on Microsoft UF2 specification
"""
import sys
import struct

# UF2 constants
UF2_MAGIC_START0 = 0x0A324655  # "UF2\n"
UF2_MAGIC_START1 = 0x9E5D5157  # Magic 2
UF2_MAGIC_END = 0x0AB16F30    # End magic
UF2_FLAG_FAMILY_ID = 0x00002000

# RP2350 constants (Pico 2)
RP2350_FAMILY_ID = 0xE48BFF59  # RP2350-ARM-S (Secure)
FLASH_START = 0x10000000
BLOCK_SIZE = 256  # Data per UF2 block

def bin_to_uf2(bin_path, uf2_path):
    with open(bin_path, 'rb') as f:
        data = f.read()
    
    num_blocks = (len(data) + BLOCK_SIZE - 1) // BLOCK_SIZE
    blocks = []
    
    for block_no in range(num_blocks):
        start = block_no * BLOCK_SIZE
        end = min(start + BLOCK_SIZE, len(data))
        block_data = data[start:end]
        
        # Pad to 256 bytes
        block_data = block_data + b'\x00' * (BLOCK_SIZE - len(block_data))
        
        # Build UF2 block (512 bytes total)
        flags = UF2_FLAG_FAMILY_ID
        addr = FLASH_START + start
        
        header = struct.pack('<IIIIIIII',
            UF2_MAGIC_START0,
            UF2_MAGIC_START1,
            flags,
            addr,
            BLOCK_SIZE,
            block_no,
            num_blocks,
            RP2350_FAMILY_ID
        )
        
        # Pad header+data to 476 bytes, then add end magic
        padding = b'\x00' * (476 - len(block_data))
        footer = struct.pack('<I', UF2_MAGIC_END)
        
        blocks.append(header + block_data + padding + footer)
    
    with open(uf2_path, 'wb') as f:
        for block in blocks:
            f.write(block)
    
    print(f"Converted {len(data)} bytes to {len(blocks)} UF2 blocks")
    print(f"Output: {uf2_path}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input.bin> <output.uf2>")
        sys.exit(1)
    bin_to_uf2(sys.argv[1], sys.argv[2])
