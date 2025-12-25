# PROJECT CONSTRAINTS

## 1. Development Guidelines
- **Real-Time Criticality**: 
  - **Do NOT use Python** for any real-time critical path (e.g., SPI communication, Control Loops, Sensor Fusion). Python's GC and GIL introduce unpredictable latency.
  - **Preferred Languages**: C++ (ROS 2 standard) or Rust (Safety/Performance).
  - Python is acceptable *only* for non-critical tooling, visualization, or configuration scripts.
- **Agent Autonomy**: Agents (like Antigravity) are authorized to:
  - Create/Delete files and directories.
  - Install system packages (apt, pip) as needed.
  - Edit configuration files.
  - Run build commands and flash firmware.
- **Directory Hygiene**: Agents **MUST keep directories tidy** at all times. Remove obsolete files, organize code properly, and maintain a clean project structure.
- **Privacy & Documentation**:
  - **Public Docs (README)**: **MUST NOT** contain private IPs, device names, or passwords.
  - **Private Context**: Use `HANDOVER_CONTEXT.md` (which is ignored or internal) for sensitive deployment details.
  - **Tidy up directories** at all times (remove obsolete files, organize structure, clean up build artifacts).

## 2. Naming Conventions (**CRITICAL**)

- **Project Name**: ALWAYS use "ALTAIR" or "altair".
- **Forbidden Terms**: NEVER use "voliro" or other project names.
- **Components**:
  - `altair_controller`
  - `altair_description`
  - `altair_gcs`

## 3. Hardware Constraints

- **Platform**: Raspberry Pi 4 (Ubuntu 22.04 + ROS 2 Humble).
- **Actuators**: 6x Rotors, 6x Dynamixel Servos (Tilt).
- **Sensors**: Pico-based Hub (IMU, Baro, Mag) over SPI.
- **Pico SPI**: **MUST use PIO SPI Slave** (Hardware peripheral is unreliable for Slave CS). DMA + 32-bit PIO ASM loop required for sync.

## 3. Software Architecture

- **Control**: Hierarchical (NMPC High-Level + Pico Rate PID Low-Level).
- **Solver**: Acados (Real C-Code integration).
- **Middleware**: ROS 2 Humble.
- **Agent-Driven Workflow**: The Agent must automate ALL tasks possible (Setup, Gen, Build) via scripts/SSH. User only acts when Physical Interaction is required (Flashing/Power).

## 4. Development Rules

- **Full Dev**: No mocks. Use real sensors/solvers where possible.
- **Generation**: User handles Acados code generation via provided scripts.
- **Verification**: Always verify "Proxima" architecture compliance.

## 5. Remote Access

- **RPi4_1 SSH**: `ssh konn@<tailscale IP>` (e.g., `ssh konn@100.x.x.x`)

## 6. Git Workflow

- **Frequent Commits**: Agents should push to git frequently to preserve progress and enable collaboration.
- **Branch-Based Development**: Agents **MUST create feature branches** for all work. **NEVER push directly to `main`**.
- **Temporary/Test Files**: OK to push to feature branches, but **NEVER merge** these to `main`.
- **Merge Criteria**: Only merge a branch to `main` when **all tasks for that branch are fully completed** and verified.
- **No Fast-Forward**: Fast-forward merges are **strictly NOT allowed**. Always use `git merge --no-ff` to preserve branch history.
- **Agent Context & Handover**:
  - **Knowledge Preservation**: Agents **MUST** update `HANDOVER_CONTEXT.md` (or relevant docs) with new findings, fixes, or architectural decisions before ending a session.
  - **Goal**: Ensure future agents (in different sessions) have access to the latest state and "tribal knowledge" to avoid re-discovering resolved issues.
  - **No Agent Context Files**: Agent context documents (e.g., `PROJECT_TASKS.md`, `.agent/` files, markdown files for inter-agent communication) **MUST NOT be pushed** to git. Add them to `.gitignore`.
- **Temporary & Test Files**:
  - **Marking**: Agents must clearly mark test files (e.g., `test_*.c`, `temp_Folder/`).
  - **Git Rule**: Temp files can be pushed to feature branches but **NEVER merged to `main`**.
  - **Cleanup**: Before merging to `main`, test files must be **deleted** or **integrated** into proper source code.
  - **Exception**: Practical utility scripts (e.g., `verify_connections.sh`) are allowed on `main`.
