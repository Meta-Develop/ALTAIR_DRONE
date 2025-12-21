# ALTAIR: 12-DOF Fully Actuated Hexacopter Project

A comprehensive project for a fully actuated 12-DOF hexacopter (Voliro-type), providing the complete mechanical design, electrical schematics, and distributed control software.

## System Overview

The ALTAIR project integrates custom hardware, advanced electronics, and distributed software to achieve fully actuated flight.

- **Hardware**: Custom 12-DOF hexacopter frame with tilting rotors.
- **Electronics**: Distributed computing architecture using Raspberry Pi 4s and Pico.
- **Software**: ROS 2 Humble based control stack with real-time firmware.

## Hardware

The drone features a **Voliro-type** mechanism, allowing for 12 degrees of freedom (DOF).

- **Frame**: Custom hexacopter design.
- **Actuation**: 6x Propellers for thrust, 6x Servos for vectoring.

## Electronics

The electrical system is built around a 3-Node architecture:

- **Node A (RPi Pico)**: **Real-time I/O Hub**
  - **Role**: Reads Sensors, Drives Motors.
  - **Components**: RP2040, BNO055 (I2C), 6x ESCs (DShot).
- **Node B (RPi 4 #1)**: **Bridge & Fusion**
  - **Role**: Sensor Fusion (EKF), Safety Mux, Servo Driver.
  - **Components**: Raspberry Pi 4, Servo Controller.
- **Node C (RPi 4 #2)**: **High-Level Control**
  - **Role**: NMPC / PID Control.
  - **Components**: Raspberry Pi 4.

## Network Configuration

The system uses a specific network topology to isolate the control node (Node C) for low-latency performance.

### IP Addresses

| Node                | Interface         | IP Address        | Note              |
| :------------------ | :---------------- | :---------------- | :---------------- |
| **Node B (RPi4_1)** | `wlan0` (WiFi)    | `192.168.137.xxx` | Dynamic (Hotspot) |
|                     | `eth0` (Ethernet) | `192.168.10.1`    | Static Gateway    |
|                     | Tailscale         | `100.105.97.114`  | `konnpi-1`        |
| **Node C (RPi4_2)** | `eth0` (Ethernet) | `192.168.10.2`    | Static Client     |
|                     | Tailscale         | `100.111.243.32`  | `konnpi2`         |
|                     | `wlan0` (WiFi)    | **DISABLED**      | For performance   |

### Connectivity Guide

**1. Accessing Node C (Tailscale - Recommended)**
Direct access via VPN (works from PC).

```bash
ssh konn@100.111.243.32
```

**2. Accessing Node C (Via Jump Host)**
Tunnel through Node B if Tailscale is unavailable or for local checks.

```bash
ssh -J konn@100.105.97.114 konn@192.168.10.2
```

## Directory Structure

- `hardware/`: CAD files and mechanical specifications.
- `electric/`: Wiring diagrams, schematics, and PCB designs.
- `software/`: Source code for all nodes.
  - `pico_firmware/`: C/C++ firmware for Node A.
  - `RPi4_1_ws/`: ROS 2 workspace for Node B (Bridge).
  - `RPi4_2_ws/`: ROS 2 workspace for Node C (Controller).
  - `gcs_tools_ws/`: ROS 2 workspace for GCS (PC).
  - `common/`: Shared resources (URDF, etc.).
  - `scripts/`: Utility scripts.

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

### 1. Flash Firmware (Remote)

Remote flashing via the Debug Probe is fully supported.

```bash
~/ALTAIR_DRONE/software/scripts/verify_connections.sh
```

Or manually:

```bash
# Flash Sensors (Pico 2 A)
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000; program software/firmware/pico_sensors/build/pico_sensors.elf verify reset exit"
# Flash Actuators (Pico 2 B)
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000; program software/firmware/pico_actuators/build/pico_actuators.elf verify reset exit"
```

### 2. Monitoring (Alive Check)

Check the Onboard LED (Green) on each Pico:

- **Pico 2 A (Sensors)**: Blinks fast (5Hz) -> Loop running.
- **Pico 2 B (Actuators)**: Blinks slow (2Hz) -> Loop running.

### 3. Start Bridge (Node B)

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
