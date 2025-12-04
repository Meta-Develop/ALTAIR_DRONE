import os
from dynamixel_sdk import *

# Control table address
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
PROTOCOL_VERSION        = 2.0
BAUDRATES = [4000000, 57600, 1000000, 115200, 9600]
DEVICENAME = '/dev/ttyUSB1'

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print("Failed to open the port")
    quit()

for baud in BAUDRATES:
    print(f"Checking Baudrate: {baud}")
    if not portHandler.setBaudRate(baud):
        print(f"Failed to change the baudrate to {baud}")
        continue

    print("Scanning for IDs 1-6...")
    found_any = False
    for dxl_id in range(1, 7):
        dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)
        if dxl_comm_result == COMM_SUCCESS:
            print(f"[ID:{dxl_id:03d}] ping succeeded. Model Number: {dxl_model_number}")
            found_any = True
    
    if found_any:
        print(f"SUCCESS: Found servos at {baud} baud.")
        break
    else:
        print(f"No servos found at {baud} baud.")

portHandler.closePort()
