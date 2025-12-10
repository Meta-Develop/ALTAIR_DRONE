import serial
import time

try:
    ser = serial.Serial('COM3', 115200, timeout=1)
    print("Opened COM3 successfully.")
    
    start = time.time()
    while time.time() - start < 10:
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                print(line)
            except Exception as e:
                print(f"Read Error: {e}")
        time.sleep(0.01)
        
    ser.close()
    print("Closed COM3.")

except Exception as e:
    print(f"Failed to open COM3: {e}")
