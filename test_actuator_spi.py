import spidev
import time
import struct

# Actuator Packet (Send): Magic(2), Throttle(12), Flags(1), Cksum(1) = 16 bytes
# Telemetry Packet (Recv): Magic(2), RPM(12), Volt(12), Curr(12), Temp(6), Stat(1), Cksum(1) = 46 bytes?
# Let's check struct size in main.c
# ActuatorPacket: 2 + 12 + 1 + 1 = 16 bytes.
# TelemetryPacket: 2 + 12 + 12 + 12 + 6 + 1 + 1 = 46 bytes.

# SPI Config
spi = spidev.SpiDev()
spi.open(0, 0) # SPI0, CE0
spi.max_speed_hz = 1000000 
spi.mode = 0

def calculate_checksum(data):
    xor = 0
    for b in data:
        xor ^= b
    return xor

def test():
    print("Testing Pico 2B SPI Connection...")
    
    # Create Command: Idle (0 throttle)
    cmd = bytearray(16)
    cmd[0] = 0xBA
    cmd[1] = 0xBE
    # Throttle 0
    # Flags = 0 (Disarmed)
    cmd[15] = calculate_checksum(cmd[:15])
    
    # Transaction
    # We send 16 bytes, but need to read 46 bytes?
    # SPI is full duplex. 
    # If we want to read 46 bytes, we must xfer 46 bytes.
    # We pad our command to 46 bytes.
    
    tx_buf = cmd + bytearray(46 - 16)
    
    # Assert CS (Software manual if needed, but SpiDev handles CE0)
    # However, our Pico expects CS framing.
    
    rx_buf = spi.xfer2(list(tx_buf))
    
    bytes_rx = bytearray(rx_buf)
    
    print(f"TX: {tx_buf.hex()}")
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
