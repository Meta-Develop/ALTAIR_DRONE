# ALTAIR Hexacopter Control System

A distributed control system for a fully actuated 12-DOF hexacopter (Voliro-type), built with ROS 2 Humble and Raspberry Pi Pico.

## Architecture

The system consists of three main nodes:

*   **Node A (RPi Pico)**: Real-time I/O hub. Handles BNO055 IMU reading, DShot motor control, and ESC telemetry.
*   **Node B (RPi 4 #1)**: Bridge node. Handles sensor fusion (EKF), actuator dispatching, and communication with the Ground Control Station (GCS).
*   **Node C (RPi 4 #2)**: Control node. Runs the high-level controller (NMPC/PID) and generates actuator commands.

## Directory Structure

*   `software/pico_firmware/`: C/C++ firmware for the Raspberry Pi Pico (Node A).
*   `software/RPi4_1_ws/`: ROS 2 workspace for Node B (Bridge).
*   `software/RPi4_2_ws/`: ROS 2 workspace for Node C (Controller).
*   `software/gcs_tools_ws/`: ROS 2 workspace for GCS (PC).
*   `software/common/`: Shared resources (URDF, etc.).
*   `software/scripts/`: Utility scripts.

## Installation

### Prerequisites
*   ROS 2 Humble
*   Micro-ROS Agent
*   Raspberry Pi Pico SDK

### Build Instructions

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/your-org/ALTAIR_DRONE.git
    cd ALTAIR_DRONE
    ```

2.  **Build Pico Firmware (Node A)**:
    ```bash
    cd software/pico_firmware
    mkdir build && cd build
    cmake ..
    make
    ```

3.  **Build Node B (RPi 4 #1)**:
    ```bash
    cd software/RPi4_1_ws
    colcon build
    source install/setup.bash
    ```

4.  **Build Node C (RPi 4 #2)**:
    ```bash
    cd software/RPi4_2_ws
    colcon build
    source install/setup.bash
    ```

5.  **Build GCS (PC)**:
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
MIT License