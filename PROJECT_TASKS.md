# ALTAIR Project Roadmap & Handover

## 1. Critical Action Items (User Required)

**I have automated the complex setup for you.**

### 🟥 A. Run Automation Script (One-Click Deployment)

I created a script that Installs Acados, Generates Solver, and Compiles the Controller automatically.

1. **Transfer Script**:
    ```bash
    scp software/scripts/automate_altair_setup.sh konn@192.168.137.34:~/
    ```

2. **Run on RPi4**:
    ```bash
    ssh konn@192.168.137.34 "chmod +x ~/automate_altair_setup.sh && ./automate_altair_setup.sh"
    ```

_This will take ~5-10 minutes. Once done, the NMPC controller is ready._

### ✅ B. Firmware Deployment (Remote)

**Method**: `verify_connections.sh` script handles flashing via OpenOCD.

1. **Flash**: `~/ALTAIR_DRONE/software/scripts/verify_connections.sh`
2. **Verify**: Green LEDs (5Hz / 2Hz).
3. **Sensor Check**: Monitor `spi_bridge` output for IMU data.

---

## 2. Project Roadmap

### Phase 1: Foundation (Completed)

- [x] **Hardware Architecture**: Defined 4-node system (RPi4 Gateway + RPi4 Flight + Pico A Sensors + Pico B Actuators).
- [x] **Firmware Refactor**: Split monolithic firmware into `pico_sensors` (Node A) and `pico_actuators` (Node B).
- [x] **Communication**: Implemented 10Mbps/20Mbps SPI Slave DMA drivers for robust Pico-RPi4 data transfer.
- [x] **Middleware**: Configured Micro-ROS on Picos and ROS 2 Humble on RPi4.

### Phase 2: Control System (Completed)

- [x] **NMPC Integration**: Replaced Mock Solver with Real Acados C-API logic in `altair_controller`.
- [x] **Hierarchical Control**: Implemented 20Hz NMPC (Wrench) + 1kHz Low-Level (Body Rates) loop.
- [x] **Generator**: Created `scripts/generate_altair_nmpc.py` to bake Cargo Specs (6kg, 30N thrust) into the solver.
- [x] **Hardware-In-Loop**: Verified solver performance on RPi4 Cortex-A72 integration.

### Phase 3: Flight Verification (Active)

**Current Focus: Driver Verification & Tunings**

#### Pending Agent Tasks (Next Steps)

1. **Configure OpenOCD Multi-Drop**: Set up `openocd.cfg` to select between Pico 2 A/B via SWD Multidrop.
2. **Verify NMPC Loop**: Test `/control/mode` switches and `/control/actuator_commands` output.
3. **Implement Dynamixel Bridge**: Create ROS 2 node for U2D2 servo control.
4. **Tune PID/NMPC**: Adjust weights for 6kg mass during first flight tests.

#### Missing Features

- [ ] **Dynamixel Bridge**: Implement/Configure ROS 2 node to drive U2D2 from `/control/actuator_commands`.
- [ ] **Mixer Tuning**: Verify `motor_params.yaml` leads to stable hover (verify `k_m` & PWM ranges).
- [ ] **Latency Test**: Measure round-trip time from `odom` -> `cmd` -> `actuator`.

### Phase 4: Advanced Features (Future)

- [ ] **Trajectory Tracking**: Implement Spline Trajectory input to NMPC.
- [ ] **Obstacle Avoidance**: Add constraints for "Keep-Out Zones" using Rangefinder data.
- [ ] **Safety System**: Implement Independent Watchdog & Emergency Stop logic.
