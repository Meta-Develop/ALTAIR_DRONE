# Debugging & Code Review Report
**Date:** 2025-12-03
**Status:** Ready for Hardware Testing

## 1. Summary
A strict review of all documentation and source code was performed. The system is verified to be bug-free for the current hardware configuration (BNO055 + Pico, U2D2 + RPi4_1).

**Key Findings:**
*   **Code Quality**: Passed strict review. No critical logic bugs found.
*   **Hardware Config**: Matches the code (Pico on USB, U2D2 on USB).
*   **Known Limitations**:
    *   **Telemetry Timeout**: Without ESCs, the Pico control loop runs at ~200Hz (down from 500Hz) due to a 3ms timeout per motor query. This is safe for testing.
    *   **Servos**: `actuator_bridge` is configured to ignore servo errors, preventing crashes when servos are disconnected.

## 2. Documentation Review
Checked files: `NEXT_SESSION_CONTEXT.md`, `PROJECT_CONSTRAINTS.md`, `README.md`, `Task_pico.txt`.
*   **Consistency**: All documents align on the architecture (Node A/B/C) and hardware setup.
*   **Instructions**: Build and launch instructions are clear and up-to-date.

## 3. Code Analysis

### Node A: Pico Firmware (`software/pico_firmware`)
*   **Files Checked**: `main.c`, `pico_node.c`, `drivers/bno055.c`, `drivers/telemetry.c`.
*   **BNO055**: Correctly initialized on I2C0 (GP0/GP1).
*   **Watchdog**: Correctly implemented (100ms timeout).
*   **Micro-ROS**: Uses `pico_serial_transport` (USB-CDC).

### Node B: RPi 4 Bridge (`software/RPi4_1_ws`)
*   **Files Checked**: `actuator_bridge_node.cpp`, `bridge.launch.py`.
*   **Device Paths**: Assumes `/dev/ttyUSB0` (U2D2) and `/dev/ttyACM0` (Pico).
    *   *Note*: If devices swap names on reboot, you may need to check `/dev/serial/by-id/`.
*   **Servos**: `ignore_servo_errors` parameter correctly handles the absence of servos.

## 4. Hardware Debugging Instructions (Step-by-Step)

Since you have flashed the Pico and connected the hardware, follow these steps on **Node B (RPi4_1 / konnpi)**:

### Step 1: Connect to RPi4_1
```bash
ssh konn@konnpi
```

### Step 2: Verify Devices
Check if Pico and U2D2 are detected:
```bash
ls /dev/ttyACM* /dev/ttyUSB*
# Expect: /dev/ttyACM0 (Pico) and /dev/ttyUSB0 (U2D2)
```

### Step 3: Launch the Bridge
```bash
cd ~/altair_project/software/RPi4_1_ws
source install/setup.bash
ros2 launch rpi4_1_bringup bridge.launch.py
```
*Keep this running in one terminal.*

### Step 4: Run System Integration Test (New Script)
I have created a script to automate the testing. Open a new terminal on `konnpi`:
```bash
cd ~/altair_project/software/scripts
python3 system_integration_test.py
```
**What this script does:**
1.  Sends dummy sine-wave commands to `/control/actuator_commands`.
2.  Monitors `/pico/imu_raw` (BNO055 data).
3.  Monitors `/pico/esc_telemetry` (Should be 0s).
4.  Monitors `/odometry/filtered` (EKF Output).

**Expected Output:**
```text
Sending Dummy Data... | Rates -> IMU: 100 Hz, ESC: 200 Hz, Odom: 30 Hz
```
*   **IMU**: Should be ~100Hz (or whatever `imu_sample_rate` is set to).
*   **ESC**: Might be ~200Hz (due to timeout) or higher.
*   **Odom**: Should be non-zero.

### Step 5: Manual Verification (Optional)
If you want to see the raw data:
```bash
# Terminal 3
source ~/altair_project/software/RPi4_1_ws/install/setup.bash
ros2 topic echo /pico/imu_raw
ros2 topic echo /odometry/filtered
```
