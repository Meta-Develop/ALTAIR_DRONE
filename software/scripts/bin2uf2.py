#!/usr/bin/env python3
"""
bin2uf2.py - Convert binary file to UF2 format for RP2040

Usage: python bin2uf2.py input.bin output.uf2
"""
import sys
import struct

UF2_MAGIC_START0 = 0x0A324655  # "UF2\n"
UF2_MAGIC_START1 = 0x9E5D5157  # Randomly selected
UF2_MAGIC_END = 0x0AB16F30    # Randomly selected

# RP2040 family ID
RP2040_FAMILY_ID = 0xe48bff56

def convert_to_uf2(input_file, output_file, flash_addr=0x10000000):
    with open(input_file, 'rb') as f:
        data = f.read()
    
    blocks = []
    data_len = len(data)
    num_blocks = (data_len + 255) // 256
    
    for i in range(num_blocks):
        chunk = data[i*256:(i+1)*256]
        # Pad to 256 bytes
        chunk += b'\x00' * (256 - len(chunk))
        
        flags = 0x00002000  # familyID present
        block = struct.pack('<IIIIIIII',
            UF2_MAGIC_START0,
            UF2_MAGIC_START1,
            flags,
            flash_addr + i * 256,
            256,
            i,
            num_blocks,
            RP2040_FAMILY_ID
        )
        block += chunk
        block += b'\x00' * (512 - 32 - 256 - 4)
        block += struct.pack('<I', UF2_MAGIC_END)
        blocks.append(block)
    
    with open(output_file, 'wb') as f:
        for block in blocks:
            f.write(block)
    
    print(f"Converted {data_len} bytes to {len(blocks)} UF2 blocks")
    print(f"Output: {output_file}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python bin2uf2.py input.bin output.uf2")
        sys.exit(1)
    
    convert_to_uf2(sys.argv[1], sys.argv[2])
