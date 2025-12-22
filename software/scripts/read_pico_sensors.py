import time
import os
import struct
import sys

# Pin Defs (BCM)
CS_PIN = 8
CLK_PIN = 11
MOSI_PIN = 10
MISO_PIN = 9

GPIO_PATH = "/sys/class/gpio"

def export_pin(pin, direction):
    path = os.path.join(GPIO_PATH, f"gpio{pin}")
    if not os.path.exists(path):
        try:
            with open(os.path.join(GPIO_PATH, "export"), "w") as f:
                f.write(str(pin))
        except OSError:
            pass # print(f"Warning: Failed to export {pin}, maybe already exported.")
    time.sleep(0.1) # Wait for creation
    with open(os.path.join(path, "direction"), "w") as f:
        f.write(direction)

class BitBangSPI:
    def __init__(self):
        # print("Exporting pins...")
        export_pin(CS_PIN, "out")
        export_pin(CLK_PIN, "out")
        export_pin(MOSI_PIN, "out")
        export_pin(MISO_PIN, "in")
        
        self.f_cs = open(f"{GPIO_PATH}/gpio{CS_PIN}/value", "w")
        self.f_clk = open(f"{GPIO_PATH}/gpio{CLK_PIN}/value", "w")
        self.f_mosi = open(f"{GPIO_PATH}/gpio{MOSI_PIN}/value", "w")
        self.f_miso = open(f"{GPIO_PATH}/gpio{MISO_PIN}/value", "r")
        
        # Idle State
        self.set_pin(self.f_cs, 1)
        self.set_pin(self.f_clk, 0)
        self.set_pin(self.f_mosi, 0)
        
    def set_pin(self, f, val):
        f.write("1" if val else "0")
        f.seek(0)
        
    def get_pin(self, f):
        f.seek(0)
        val = f.read(1)
        return 1 if val == "1" else 0

    def transfer_byte(self, byte_out):
        byte_in = 0
        for i in range(8):
            # MSB First
            bit_out = (byte_out >> (7 - i)) & 1
            self.set_pin(self.f_mosi, bit_out)
            
            # Setup Time
            time.sleep(0.002) 

            # Clock High (Sample MISO)
            self.set_pin(self.f_clk, 1)
            time.sleep(0.002) # Strobe Delay

            # Read MISO
            if self.get_pin(self.f_miso):
                byte_in |= (1 << (7 - i))
            
            # Clock Low (Next Bit)
            self.set_pin(self.f_clk, 0)
            time.sleep(0.002) # Hold Time
            
        return byte_in

    def transfer(self, tx_data):
        rx_data = []
        # CS Low
        self.set_pin(self.f_cs, 0)
        # Delay for Pico ISR
        time.sleep(0.005) 
        
        for val in tx_data:
            rx_data.append(self.transfer_byte(val))
            
        # CS High
        self.set_pin(self.f_cs, 1)
        return rx_data

    def close(self):
        self.f_cs.close()
        self.f_clk.close()
        self.f_mosi.close()
        self.f_miso.close()

def main():
    spi = BitBangSPI()
    print("Reading Real Sensor Data from Pico (Auto-Align)...", flush=True)
    
    try:
        while True:
            # Read extra bytes to allow for shift
            tx_dummy = [0x00] * 40 
            raw = spi.transfer(tx_dummy)
            
            # Search for Sync Byte 0xAA
            start_idx = -1
            for i in range(len(raw) - 30):
                if raw[i] == 0xAA:
                    start_idx = i
                    break
            
            if start_idx >= 0:
                # Check End Byte (at start + 29)
                end_idx = start_idx + 29
                if raw[end_idx] == 0xBB:
                    payload = bytes(raw[start_idx+1 : end_idx]) # 28 bytes
                    try:
                        # unpack '<ffffffI'
                        unpacked = struct.unpack('<ffffffI', payload)
                        ax, ay, az, gx, gy, gz, ts = unpacked
                        print(f"ALIGN@{start_idx} | TS: {ts} | Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}) | Gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f})")
                    except struct.error as e:
                         print(f"ALIGN@{start_idx} | Unpack Error: {e}")
                else:
                     hex_str = ' '.join([f'{b:02X}' for b in raw[start_idx:start_idx+5]])
                     print(f"ALIGN@{start_idx} | End Byte Mismatch (Expected BB @ {end_idx}, got {raw[end_idx]:02X}) | Raw: {hex_str}...")
            else:
                hex_str = ' '.join([f'{b:02X}' for b in raw[:10]])
                print(f"Sync Fail: {hex_str}...")
                
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nExiting...")
        spi.close()

if __name__ == "__main__":
    main()
