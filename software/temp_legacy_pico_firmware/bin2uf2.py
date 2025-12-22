#!/usr/bin/env python3
"""Convert a binary file to UF2 format for RP2040."""
import struct
import sys
import os

UF2_MAGIC_START0 = 0x0A324655  # "UF2\n"
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
UF2_FLAG_FAMILY_ID = 0x00002000
RP2040_FAMILY_ID = 0xE48BFF56

def convert_bin_to_uf2(bin_path, uf2_path, start_addr=0x10000000):
    with open(bin_path, 'rb') as f:
        data = f.read()
    
    # Split into 256-byte chunks
    chunks = []
    for i in range(0, len(data), 256):
        chunk = data[i:i+256]
        # Pad to 256 bytes
        if len(chunk) < 256:
            chunk = chunk + b'\x00' * (256 - len(chunk))
        chunks.append(chunk)
    
    num_blocks = len(chunks)
    
    with open(uf2_path, 'wb') as f:
        for block_no, chunk in enumerate(chunks):
            # Header (32 bytes)
            header = struct.pack('<IIIIIIII',
                UF2_MAGIC_START0,   # magicStart0
                UF2_MAGIC_START1,   # magicStart1
                UF2_FLAG_FAMILY_ID, # flags
                start_addr + block_no * 256,  # targetAddr
                256,                # payloadSize
                block_no,           # blockNo
                num_blocks,         # numBlocks
                RP2040_FAMILY_ID    # familyID (RP2040)
            )
            
            # Data (476 bytes: 256 payload + 220 padding)
            payload = chunk + b'\x00' * 220
            
            # Footer (4 bytes)
            footer = struct.pack('<I', UF2_MAGIC_END)
            
            # Total block: 512 bytes
            f.write(header + payload + footer)
    
    print(f"Created {uf2_path}: {num_blocks} blocks, {num_blocks * 512} bytes")
    return True

if __name__ == "__main__":
    bin_file = sys.argv[1] if len(sys.argv) > 1 else "pico_spi.bin"
    uf2_file = os.path.splitext(bin_file)[0] + ".uf2"
    
    if not os.path.exists(bin_file):
        print(f"Error: {bin_file} not found")
        sys.exit(1)
    
    convert_bin_to_uf2(bin_file, uf2_file)
