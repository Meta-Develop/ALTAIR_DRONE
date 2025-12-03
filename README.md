# ALTAIR: 12-DOF Fully Actuated Hexacopter Project

A comprehensive project for a fully actuated 12-DOF hexacopter (Voliro-type), providing the complete mechanical design, electrical schematics, and distributed control software.

## System Overview

The ALTAIR project integrates custom hardware, advanced electronics, and distributed software to achieve fully actuated flight.

*   **Hardware**: Custom 12-DOF hexacopter frame with tilting rotors.
*   **Electronics**: Distributed computing architecture using Raspberry Pi 4s and Pico.
*   **Software**: ROS 2 Humble based control stack with real-time firmware.

## Hardware

The drone features a **Voliro-type** mechanism, allowing for 12 degrees of freedom (DOF).
*   **Frame**: Custom hexacopter design.
*   **Actuation**: 6x Propellers for thrust, 6x Servos for vectoring.

## Electronics

The electrical system is built around a 3-Node architecture:

*   **Node A (RPi Pico)**: **Real-time I/O Hub**
    *   **Role**: Reads Sensors, Drives Motors.
    *   **Components**: RP2040, BNO055 (I2C), 6x ESCs (DShot).
*   **Node B (RPi 4 #1)**: **Bridge & Fusion**
    *   **Role**: Sensor Fusion (EKF), Safety Mux, Servo Driver.
    *   **Components**: Raspberry Pi 4, Servo Controller.
*   **Node C (RPi 4 #2)**: **High-Level Control**
    *   **Role**: NMPC / PID Control.
    *   **Components**: Raspberry Pi 4.

## Directory Structure

*   `hardware/`: CAD files and mechanical specifications.
*   `electric/`: Wiring diagrams, schematics, and PCB designs.
*   `software/`: Source code for all nodes.
    *   `pico_firmware/`: C/C++ firmware for Node A.
    *   `RPi4_1_ws/`: ROS 2 workspace for Node B (Bridge).
    *   `RPi4_2_ws/`: ROS 2 workspace for Node C (Controller).
    *   `gcs_tools_ws/`: ROS 2 workspace for GCS (PC).
    *   `common/`: Shared resources (URDF, etc.).
    *   `scripts/`: Utility scripts.

## Installation

### 1. Build Node A (RPi Pico)
```bash
cd software/pico_firmware
mkdir build
cd build
cmake ..
make
```

### 2. Build Node B (RPi 4 #1)
```bash
cd software/RPi4_1_ws
colcon build
source install/setup.bash
```

### 3. Build Node C (RPi 4 #2)
```bash
cd software/RPi4_2_ws
colcon build
source install/setup.bash
```

### 4. Build GCS (PC)
```bash
cd software/gcs_tools_ws
colcon build
source install/setup.bash
```

## Usage

### 1. Flash Firmware
Connect the Pico in BOOTSEL mode and copy `software/pico_firmware/build/pico_firmware.uf2` to the drive.

### 2. Start Bridge (Node B)
```bash
ros2 launch rpi4_1_bringup bridge.launch.py
```

### 3. Start Controller (Node C)
```bash
ros2 launch altair_controller controller.launch.py
```

### 4. Start GCS (PC)
```bash
ros2 launch altair_gcs_gui gcs.launch.py
```

## License
Copyright (c) 2025 ALTAIR Drone Project. All Rights Reserved.
Proprietary and Confidential. Not Open Source.