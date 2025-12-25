# Agent Handover Context

## Critical Lessons Learned (Dec 25, 2024)
1. **SPI Mode Mismatch (ROOT CAUSE FIXED)**:
   - **Issue declaration**: Initialization of ISM330DHCX failed (returning `0x00`).
   - **Cause**: Pico SDK defaults to **SPI Mode 0** (CPOL=0, CPHA=0). ISM330DHCX requires **SPI Mode 3** (CPOL=1, CPHA=1).
   - **Fix**: Added `spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);` to firmware.
   - **Result**: All 4 sensors now initialize successfully:
     - ISM330DHCX (`0x6B`)
     - MMC5983MA (`0x30`)
     - BMP388 (`0x50`)
     - VL53L4CX (`0xEB`)

2. **Physical Wiring Swap Resolved**:
   - Confirmed Actuator Pico was on Sensor Bus (SPI0) and vice-versa.
   - **Action**: Firmware reflashed to matching devices.
   - **Sensor Pico (SPI0)**: `F6DE...` (RP2350) -> Running `pico_sensors` (Mode 3 fixed).
   - **Actuator Pico (SPI1)**: `E018...` (Pico 2) -> Running `pico_actuators` (Simple Test FW).

3. **Current Status (Blocking)**:
   - **ROS2 Integration**: **SUCCESS** ✅
     - Actuators: **SUCCESS** ✅ (`actuator_spi_node` publishing telemetry)
     - Sensors: **SUCCESS** ✅ (`spi_bridge_cpp` publishing at ~90Hz using `libgpiod`)
   - **Next Step**: Tune Magic detection and optimize loop timing for 1kHz.

## Critical Lessons Learned (Dec 25, 2024) - Part 2
1. **SPI Slave Data Corruption (Code Fix)**:
   - **Issue**: RPi4 received correct number of bytes but content was garbage/drifted.
   - **Cause 1 (Critical)**: `spi_slave.pio` ASM loop was set to `set x, 6` (7 bits) instead of `31` (32 bits). This discarded 24 bits of every 32-bit DMA word.
   - **Cause 2**: DMA setup on "Falling Edge" (Start of Transaction) was too slow, causing missed bits.
   - **Fix**: 
     - Updated PIO ASM to `set x, 31`.
     - Refactored `main_dma.c` to pipeline DMA setup on the **Rising Edge** (End of Previous Transaction).

2. **Endianness/Word Swap Quirk**:
   - **Observation**: 32-bit data sent from Pico (Little Endian) arrives at RPi4 with 16-bit words swapped (e.g., `0xAA55` -> `0x55AA` in memory, but byte stream looks like `55 AA` vs `AA 55`).
   - **Resolution**: `verify_sensor_values.py` updated to detect and handle this 16-bit word swap. Firmware remains standard; RPi handles the quirk.


## Critical Lessons Learned (Dec 24, 2024)
1. **Physical Wiring Swap Discovered**:
   - Pico with **Actuator firmware** → SPI0 (Sensor bus) - Receiving CS signals ✅
   - Pico with **Sensor firmware** → SPI1 (Actuator bus) - No CS signals ❌
   - **Proof**: SPI0 test returned `0xCAFE` (Actuator magic), not `0xAA55` (Sensor magic)
   - **Fix**: Swap physical Picos OR reflash firmware to match bus assignment

2. **Firmware PIO Fixes Required**:
   - Add `pio_gpio_init(pio, 18)` and `gpio_set_dir(18, GPIO_IN)` for SCK in PIO init
   - Remove redundant `gpio_init(PIN_MISO_SLAVE)` before PIO init (causes function conflict)

## Previous Lessons (Dec 23, 2024)
3. **Hardware Fault Identified**: 
   - Symptoms: RPi4 receives packets with correct length but all content is `0x00` (Bad Header). Pico reports `CS IRQs: 0`.
   - Diagnosis: **SCK (GP18)** line is likely broken/disconnected. PIO programs hang on `wait 1 gpio 18`.
   - Action: **PHYSICAL WIRING FIX REQUIRED.**

2. **Performance Tuning**:
   - `usleep(6000)` in RPi4 driver was a major bottleneck (~160Hz cap).
   - Reduced to `usleep(50)` -> Software capable of >1000Hz (verified by blind polling logic).

3. **Software Robustness**:
   - **Blind Polling (2kHz)** on RPi4 is more robust than IRQ-based polling when hardware signals are flaky.
   - **Data Ready Handshake (GP22)**: Implemented on Pico but currently blocked by the root wiring issue.

4. **New Architecture (Batching)**:
   - Moving from 1kHz single-sample to **6.66kHz Batching** (7 samples/packet).
   - Requires protocol update (130 byte packets) and 6-Bank Notch Filter on RPi4.
   - Reference: `docs/specs/SENSOR_BRIDGE_SPEC.md`


> [!IMPORTANT]
> This file contains **PRIVATE & SENSITIVE** information (IPs, Device Names).
> **DO NOT COMMIT** this file to git if not ignored (it should be in `.gitignore`).
> **DO NOT** write this information to public documentation like `README.md`.

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
