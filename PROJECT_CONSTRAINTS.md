# Project Constraints & Agent Instructions

## 1. Development Workflow
*   **Git Usage**: **STRICT REQUIREMENT**.
    *   Commit and push to GitHub **frequently** (e.g., after every logical step or successful build).
    *   Do not accumulate large sets of changes.
    *   **ALWAYS** push before ending a session.
*   **Branching & Merging**: **STRICT REQUIREMENT**.
    *   **NEVER** commit directly to `main` (or `master`) for new features or refactoring.
    *   **Action**: Create a new branch (e.g., `feature/new-sensor`, `refactor/cleanup-code`), implement changes, and merge via Pull Request (or merge locally if working solo, but keep history clean).
*   **Deployment**: **STRICTLY via GitHub**.
    *   **Do NOT** use `scp` or manual copy to transfer code to RPi4s.
    *   **Action**: Push changes to GitHub from PC, then `git pull` on the RPi4s.
*   **Git Identity**: **STRICT REQUIREMENT**.
    *   **Name**: `Meta-Develop`
    *   **Email**: `rikuuyoo@gmail.com`
    *   **Action**: Ensure these are configured in your local git config (`git config user.name "Meta-Develop"`, `git config user.email "rikuuyoo@gmail.com"`).

## 2. Hardware & Network Configuration
*   **Node B (Raspberry Pi 4 #1)**:
    *   **Hostname**: `konnpi`
    *   **Username**: `konn`
    *   **OS**: Ubuntu 22.04 Server.
    *   **Access**: **STRICTLY** via Tailscale.
        *   **Instruction**: Do NOT rely on hostname resolution (`ssh konn@konnpi`).
        *   **Action**: Run `tailscale status` or `tailscale ip -4 konnpi` to find the IP, then `ssh konn@<IP>`.
    *   **Network**: Connects to **Windows PC via Mobile Hotspot**.
    *   **Role**: Internet Gateway & Jump Host for Node C.
    *   **Wiring**:
        *   **LAN**: Connected to Node C.
        *   **USB**: Connected to U2D2 and Pico.
*   **Node C (Raspberry Pi 4 #2)**:
    *   **Hostname**: `konnpi2`
    *   **Username**: `konn`
    *   **OS**: Ubuntu 22.04 Server.
    *   **Access**: Via Jump Host `konnpi` (`ssh -J konn@<IP konnpi> konn@<IPkonnpi2>`).
    *   **Network**: Internet via Ethernet from `konnpi`. **WiFi is OFF**.
    *   **Wiring**: Connected to Node B via LAN.
*   **GCS (Ground Control Station)**:
    *   **Host**: **This Windows PC**.
    *   **Network**: Hosts the Mobile Hotspot for Node B.

## 3. Software Requirements
*   **ROS 2 Version**: **Humble Hawksbill** (Strict).
    *   *Reason*: Compatibility with Ubuntu 22.04 Server on RPi 4.
    *   All code, scripts, and documentation must target ROS 2 Humble.

## 4. Operation Modes
*   **Mode Selection**:
    -   The system must support "Test Mode" (Hardware-in-the-loop or limited hardware) and "Actual Mode" (Full hardware).
    -   Modes must be switchable via ROS 2 interface (GCS).
    -   Default boot mode should be "Test Mode" (Safe, no telemetry timeouts) until explicitly switched.

## 5. Temporary Files & Cleanup
*   **Clean before Commit**: **STRICT REQUIREMENT**.
    *   If making temporary files or folders, **git ignore** or **delete** them before committing or pushing.
    *   Tidy up all files before starting the commit/push cycle.
