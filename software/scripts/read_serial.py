#!/usr/bin/env python3
"""
ALTAIR Serial Monitor - Enhanced Raw Serial Reader

Features:
    - Auto-detection of COM ports
    - Real-time Hz rate calculation
    - CSV logging option
    - Color-coded output

Usage:
    python read_serial.py              # Auto-detect COM port
    python read_serial.py COM3         # Specific COM port
    python read_serial.py COM3 --log   # With CSV logging
"""
import serial
import serial.tools.list_ports
import time
import sys
import os
import argparse
from datetime import datetime

# ANSI Colors
class Colors:
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    CYAN = "\033[96m"
    RESET = "\033[0m"

def find_pico_port():
    """Auto-detect Pico COM port."""
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        # Pico typically shows as "USB Serial Device" or contains "2E8A" (Raspberry Pi vendor ID)
        if "2E8A" in port.hwid or "USB Serial" in port.description:
            return port.device
    
    # Fallback: look for any ttyACM or COM port
    for port in ports:
        if "ACM" in port.device or "COM" in port.device:
            return port.device
    
    return None

def print_header():
    print("\n" + "=" * 60)
    print(f"{Colors.CYAN}  ALTAIR Raw Serial Monitor{Colors.RESET}")
    print("=" * 60 + "\n")

def main():
    parser = argparse.ArgumentParser(description='ALTAIR Serial Monitor')
    parser.add_argument('port', nargs='?', help='COM port (e.g., COM3)')
    parser.add_argument('--log', action='store_true', help='Enable CSV logging')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--duration', type=int, default=30, help='Duration in seconds (default: 30)')
    args = parser.parse_args()
    
    print_header()
    
    # Find port
    port = args.port
    if not port:
        print("Auto-detecting Pico COM port...")
        port = find_pico_port()
        if port:
            print(f"{Colors.GREEN}Found: {port}{Colors.RESET}\n")
        else:
            print(f"{Colors.RED}No Pico device found!{Colors.RESET}")
            print("\nAvailable ports:")
            for p in serial.tools.list_ports.comports():
                print(f"  - {p.device}: {p.description}")
            sys.exit(1)
    
    # CSV logging setup
    log_file = None
    if args.log:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"imu_log_{timestamp}.csv"
        log_file = open(log_filename, 'w')
        log_file.write("timestamp,type,ax,ay,az,gx,gy,gz\n")
        print(f"Logging to: {log_filename}\n")
    
    # Open serial port
    try:
        print(f"Opening {port} at {args.baud} baud...")
        ser = serial.Serial(port, args.baud, timeout=1)
        print(f"{Colors.GREEN}Connected!{Colors.RESET}\n")
    except Exception as e:
        print(f"{Colors.RED}Failed to open {port}: {e}{Colors.RESET}")
        sys.exit(1)
    
    # Stats
    packet_count = 0
    error_count = 0
    last_rate_time = time.time()
    last_rate_count = 0
    
    start = time.time()
    duration = args.duration
    
    print(f"Reading for {duration} seconds... (Ctrl+C to stop)\n")
    print("-" * 60)
    
    try:
        while time.time() - start < duration:
            if ser.in_waiting:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line:
                        packet_count += 1
                        
                        # Parse and display
                        if line.startswith("RAW,"):
                            parts = line.split(",")
                            if len(parts) == 7:
                                _, ax, ay, az, gx, gy, gz = parts
                                
                                # Log to CSV
                                if log_file:
                                    log_file.write(f"{time.time()},RAW,{ax},{ay},{az},{gx},{gy},{gz}\n")
                                
                                # Print every 10th packet
                                if packet_count % 10 == 0:
                                    print(f"{Colors.GREEN}[IMU]{Colors.RESET} Ax:{ax:>6} Ay:{ay:>6} Az:{az:>6} | Gx:{gx:>6} Gy:{gy:>6} Gz:{gz:>6}")
                        
                        elif line.startswith("ERR,"):
                            error_count += 1
                            print(f"{Colors.RED}[ERR]{Colors.RESET} {line}")
                        
                        else:
                            # Other messages
                            print(f"{Colors.YELLOW}[MSG]{Colors.RESET} {line}")
                
                except Exception as e:
                    print(f"{Colors.RED}Read Error: {e}{Colors.RESET}")
            
            # Rate calculation every second
            now = time.time()
            if now - last_rate_time >= 1.0:
                rate = (packet_count - last_rate_count) / (now - last_rate_time)
                
                # Color based on rate
                if rate >= 90:
                    rate_color = Colors.GREEN
                elif rate >= 50:
                    rate_color = Colors.YELLOW
                else:
                    rate_color = Colors.RED
                
                print(f"\n{rate_color}[RATE] {rate:.1f} Hz{Colors.RESET} | Total: {packet_count} | Errors: {error_count}")
                print("-" * 60)
                
                last_rate_count = packet_count
                last_rate_time = now
            
            time.sleep(0.001)  # Small delay to prevent CPU spin
    
    except KeyboardInterrupt:
        print("\n\nStopped by user.")
    
    finally:
        ser.close()
        if log_file:
            log_file.close()
        
        # Final summary
        elapsed = time.time() - start
        avg_rate = packet_count / elapsed if elapsed > 0 else 0
        
        print("\n" + "=" * 60)
        print(f"{Colors.CYAN}Summary:{Colors.RESET}")
        print(f"  Duration:    {elapsed:.1f} sec")
        print(f"  Packets:     {packet_count}")
        print(f"  Errors:      {error_count}")
        print(f"  Avg Rate:    {avg_rate:.1f} Hz")
        print("=" * 60 + "\n")

if __name__ == '__main__':
    main()
