# Altair Controller (Node C)

This package implements the high-level control logic for the ALTAIR Voliro-type hexacopter.

## Overview

- **Node Name:** `altair_controller`
- **Control Modes:**
    - `PID`: Cascaded PID mixer (default).
    - `NMPC`: Nonlinear Model Predictive Control (placeholder/stub).
- **Input:**
    - `/cmd_vel` (Target velocity, Reliable QoS)
    - `/odometry/filtered` (State estimate, Best Effort QoS)
- **Output:**
    - `/control/actuator_commands` (12-DOF Float32 array, Best Effort QoS)

## Architecture

The controller runs two main threads:
1.  **Control Loop (250Hz):** Handles feedback linearization, PID mixing, and watchdog safety.
2.  **MPC Thread (20Hz):** (In NMPC mode) solves the optimization problem.

## Usage

### Launching

```bash
ros2 launch altair_controller controller.launch.py mode:=PID
```

### Dependencies

- `altair_description`: Must be available in the workspace or at `~/altair_project/software/common/altair_description`.
- `Eigen3`: For matrix computations.

## System Optimization

For real-time performance on Raspberry Pi 4:

```bash
sudo ./scripts/optimize_os.sh
```
