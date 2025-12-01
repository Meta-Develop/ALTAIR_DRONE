# Remaining Tasks for ALTAIR Drone Deployment

## 1. Hardware Integration (Crucial)
The current deployment is running in **Emulation Mode**. The following hardware must be connected and verified:
- [ ] **Connect U2D2 (Dynamixel)** to RPi4_1 (Node B).
- [x] **Revert `actuator_bridge` Patch**:
    - Edit `software/RPi4_1_ws/src/actuator_bridge/src/actuator_bridge_node.cpp`.
    - Change `RCLCPP_WARN` back to `RCLCPP_ERROR` in `initDynamixel()`.
    - Ensure the node fails if servos are not detected (Safety).
- [ ] **Connect IMU & ESCs** to RPi Pico (Node A).
    - **BNO055 Wiring**:
        - SDA -> GP0 (Physical Pin 1)
        - SCL -> GP1 (Physical Pin 2)
        - VCC -> 3V3 (Physical Pin 36)
        - GND -> GND (Physical Pin 38)
    - **ESC Wiring (DShot)**:
        - Motor 1 -> GP2 (Physical Pin 4)
        - Motor 2 -> GP3 (Physical Pin 5)
        - Motor 3 -> GP4 (Physical Pin 6)
        - Motor 4 -> GP5 (Physical Pin 7)
        - Motor 5 -> GP6 (Physical Pin 9)
        - Motor 6 -> GP7 (Physical Pin 10)
        - **Grounds**: Connect all ESC signal grounds to Pico GND.
    - **ESC Telemetry (RX)**:
        - Motor 1 -> GP8 (Physical Pin 11)
        - Motor 2 -> GP9 (Physical Pin 12)
        - Motor 3 -> GP10 (Physical Pin 14)
        - Motor 4 -> GP11 (Physical Pin 15)
        - Motor 5 -> GP12 (Physical Pin 16)
        - Motor 6 -> GP13 (Physical Pin 17)
- [ ] **Verify Telemetry**:
    - Check `/pico/imu_raw` is publishing valid data.
    - Check `/pico/esc_telemetry` is publishing RPMs.
- [ ] **Build & Flash Pico Firmware**:
    - The current Pico might not have the correct firmware.
    - Build `software/pico_firmware` using the Pico SDK.
    - Flash the `.uf2` file to the Pico.

## 2. System Optimization (Node C)
- [ ] **Update URDF**:
    - `software/common/altair_description/urdf/altair.urdf` currently has placeholder values (Mass=2.0kg, only 1 rotor).
    - **Action**: Measure actual mass and arm lengths. Update URDF to reflect the 12-DOF geometry (6 rotors, tilting mechanisms).
- [ ] **CPU Isolation**:
    - Verify `isolcpus=2,3` is set in `/boot/firmware/cmdline.txt` on RPi4_2.
    - Ensure the NMPC solver is pinned to these cores.
- [ ] **Real-time Kernel**: Consider installing a PREEMPT_RT kernel if control loop jitter is too high.

## 3. Control Tuning
- [ ] **PID Tuning**: Test "Mode A" (PID Mixer) with the actual drone frame.
- [ ] **NMPC Implementation**:
    - Verify Acados/CasADi solver performance on RPi4_2.
    - Tuning weights for the MPC cost function.

## 4. Flight Safety & Startup
- [ ] **Watchdog Verification**: Test that motors disarm if Node C stops sending commands for >100ms.
- [ ] **Auto-Start**: Create `systemd` services to launch `bridge.launch.py` and `controller.launch.py` on boot.
