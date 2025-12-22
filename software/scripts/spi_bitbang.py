import time
import os

# Pin config (BCM)
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
            print(f"Warning: Failed to export {pin}, maybe already exported.")
    
    # Wait for creation
    time.sleep(0.1)
    
    # Set Direction
    with open(os.path.join(path, "direction"), "w") as f:
        f.write(direction)

def unexport_pin(pin):
    try:
        with open(os.path.join(GPIO_PATH, "unexport"), "w") as f:
            f.write(str(pin))
    except OSError:
        pass

class BitBangSPI:
    def __init__(self):
        print("Exporting pins...")
        export_pin(CS_PIN, "out")
        export_pin(CLK_PIN, "out")
        export_pin(MOSI_PIN, "out")
        export_pin(MISO_PIN, "in")
        
        # Open file handles for speed
        self.f_cs = open(f"{GPIO_PATH}/gpio{CS_PIN}/value", "w")
        self.f_clk = open(f"{GPIO_PATH}/gpio{CLK_PIN}/value", "w")
        self.f_mosi = open(f"{GPIO_PATH}/gpio{MOSI_PIN}/value", "w")
        self.f_miso = open(f"{GPIO_PATH}/gpio{MISO_PIN}/value", "r")
        
        # Idle State
        self.set_pin(self.f_cs, 1)
        self.set_pin(self.f_clk, 0)
        self.set_pin(self.f_mosi, 0)
        
        # Delay for stability
        # Python sysfs typical speed is ~1-5kHz, so no explicit delay needed for <1MHz SPI
        
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
            time.sleep(0.001) 

            # Clock High (Sample MISO)
            self.set_pin(self.f_clk, 1)
            time.sleep(0.001) # Strobe Delay

            # Read MISO
            if self.get_pin(self.f_miso):
                byte_in |= (1 << (7 - i))
            
            # Clock Low (Next Bit)
            self.set_pin(self.f_clk, 0)
            time.sleep(0.001) # Hold Time
            
        return byte_in

    def transfer(self, tx_data):
        rx_data = []
        
        # CS Low
        self.set_pin(self.f_cs, 0)
        # Delay for Pico ISR (DMA Setup)
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
        print("Closed.")

if __name__ == "__main__":
    spi = BitBangSPI()
    
    print("Reading 30 bytes from Pico (Bit-Bang)...")
    # Send dummy bytes
    tx_buf = [0x00] * 32
    rx_buf = spi.transfer(tx_buf)
    
    print("RX Data (Hex):")
    print(" ".join([f"{x:02X}" for x in rx_buf]))
    
    # Simple check for non-zero
    if any(rx_buf):
         print("SUCCESS: Received Non-Zero Data!")
    else:
         print("FAILURE: Received All Zeros.")
    
    spi.close()
