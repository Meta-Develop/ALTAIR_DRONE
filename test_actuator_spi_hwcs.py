import spidev
import time

# SPI Config
BUS = 1
DEVICE = 2 # Matches /dev/spidev1.2 (CE2 = GPIO 16)

spi = spidev.SpiDev()
spi.open(BUS, DEVICE)
spi.max_speed_hz = 100000 
spi.mode = 0
spi.no_cs = False # ✅ ENABLE HARDWARE CS (Let Kernel toggle GPIO 16)

def calculate_checksum(data):
    xor = 0
    for b in data:
        xor ^= b
    return xor

def test():
    print(f"Testing Pico (SPI1.{DEVICE}) HW CS...")
    
    cmd = bytearray(16)
    cmd[0] = 0xBA
    cmd[1] = 0xBE
    cmd[15] = calculate_checksum(cmd[:15])
    
    # 128 bytes total
    tx_buf = list(cmd + bytearray(112))
    
    rx_buf = spi.xfer2(tx_buf)
    
    bytes_rx = bytearray(rx_buf)
    
    print(f"TX: {bytes(tx_buf[:16]).hex()}...")
    print(f"RX: {bytes_rx.hex()}")
    
    if bytes_rx[0] == 0xCA and bytes_rx[1] == 0xFE:
        print("SUCCESS! Received 0xCAFE Magic.")
    else:
        print("FAILURE. Magic header missing.")

if __name__ == "__main__":
    try:
        while True:
            test()
            time.sleep(0.5)
    except KeyboardInterrupt:
        spi.close()
