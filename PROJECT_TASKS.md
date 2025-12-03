# ALTAIR Project Context & Roadmap

**Target Audience:** Future Gemini Agents & Developers
**Project:** ALTAIR (12-DOF Fully Actuated Hexacopter)
**Status:** Software Integration Phase (Post-Restructuring)

---

## 1. System Architecture

ALTAIR is a Voliro-type hexacopter capable of independent position and attitude control (12 Degrees of Freedom).

### Hardware Topology
*   **Node A (RPi Pico)**: **Real-Time I/O Hub**.
    *   *Role:* Hard real-time sensor reading and motor control.
    *   *Interfaces:* BNO055 (I2C), 6x ESCs (DShot), USB-CDC to Node B.
*   **Node B (RPi 4 #1)**: **Bridge & Fusion** (`konn@konnpi`).
    *   *Role:* Sensor fusion, hardware bridge, **Internet Gateway**.
    *   *Interfaces:* U2D2 (Dynamixel Servos), Pico (USB), Ethernet (Shares Internet to Node C).
    *   *Network:* Connects to **Windows PC via Mobile Hotspot** (Tailscale).
*   **Node C (RPi 4 #2)**: **High-Level Control** (`konn@konnpi2`).
    *   *Role:* Computationally intensive control.
    *   *Interfaces:* Ethernet (to Node B). **WiFi OFF**. Accessed via Jump Host (`konnpi`).
*   **GCS (PC)**: **Ground Control Station** (This Windows PC).
    *   *Role:* Visualization, manual override, and tuning.
    *   *Network:* Hosts Mobile Hotspot for Node B.

### Software Stack
*   **OS**: Ubuntu 22.04 (RPi 4), FreeRTOS (Pico).
*   **Middleware**: ROS 2 Humble (DDS) + Micro-ROS (Serial).
*   **Languages**: C++ (Performance nodes), Python (GUI/Tools), C (Firmware).

---

## 2. Current State

### Directory Structure
```text
ALTAIR_DRONE/
├── software/
│   ├── pico_firmware/   # [Node A] C/C++ SDK + Micro-ROS
│   ├── RPi4_1_ws/       # [Node B] ROS 2 Workspace (Bridge, EKF)
│   ├── RPi4_2_ws/       # [Node C] ROS 2 Workspace (Controller)
│   ├── gcs_tools_ws/    # [GCS]    ROS 2 Workspace (GUI, Monitor)
│   ├── common/          # Shared packages (URDF, Messages)
│   └── scripts/         # Setup scripts (install_ros2.sh, isolation)
├── README.md            # Entry point & Deployment Guide
├── PROJECT_CONSTRAINTS.md # Strict Rules & Network Config
├── PROJECT_TASKS.md     # Roadmap & Context (This File)
└── LICENSE              # MIT
```

### Recent Achievements (Session 2025-12-02)
1.  **Migration to ROS 2 Humble**:
    *   Updated scripts and documentation to target Ubuntu 22.04/Humble.
    *   Switched `micro_ros_agent` and `micro_ros_msgs` to `humble` branch.
2.  **Pico Firmware**:
    *   Cleaned and rebuilt `pico_node.uf2` (Ready in `software/pico_firmware/build/`).
3.  **RPi4_1_ws (Node B)**:
    *   Verified compilation of `actuator_bridge` and `sensor_processing`.
    *   Updated `bridge.launch.py` with `ignore_servo_errors` for HITL testing.

---

## 3. Roadmap & Tasks

### Phase 1: Hardware-in-the-Loop (HITL) Verification [CURRENT]
*   **Objective**: Verify all hardware interfaces work with the software.
*   [x] **Migrate to ROS 2 Humble**: Complete.
*   [x] **Pico Firmware**: Built `pico_node.uf2`.
*   [x] **Pico Flash**: Flash `software/pico_firmware/build/pico_node.uf2` to the Pico.
*   [x] **Hardware Connection**: Connect Pico (USB) and U2D2 (USB) to `konnpi`.
*   [ ] **System Launch**:
    ```bash
    # On konnpi
    cd software/RPi4_1_ws
    source install/setup.bash
    ros2 launch rpi4_1_bringup bridge.launch.py
    ```
*   [ ] **Verification**:
    *   Check topics: `ros2 topic list`
    *   IMU Data: `ros2 topic echo /pico/imu_raw`
    *   Actuator Bridge: Ensure no crashes.

### Phase 1.5: GCS & Debugging Tools
*   [x] **Attitude Visualization**: Create a GUI to visualize orientation (Roll/Pitch/Yaw).
*   [x] **DDS Monitor**: Create a tool to monitor all DDS topics and data flow.

### Phase 2: Control Implementation
*   [x] **Mixer**: Implement allocation matrix in `altair_controller`.
*   [ ] **PID**: Tune cascaded PID loops.
*   [ ] **NMPC**: Implement Acados solver on Node C.

### Phase 3: Electronics & Hardware
*   [ ] **PCB Design**: Carrier board for Pico.
*   [ ] **Wiring**: Secure connections.
*   [ ] **Frame Assembly**: Mount motors/servos.

### Phase 4: Flight Testing
*   [ ] Tethered Flight
*   [ ] Free Flight
*   [ ] Dynamic Maneuvers
