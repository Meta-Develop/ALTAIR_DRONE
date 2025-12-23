# PROJECT ALTAIR: SENSOR BRIDGE & SIGNAL PROCESSING SPECIFICATION

**TARGET AGENT**: Software Engineer  
**SUBJECT**: SPI Bridge Protocol, High-Freq Batching & 6-Bank Active Notch Implementation

## 1. OVERVIEW
----------------------------------------------------------------
**Objective**: Implement a high-performance sensor bridge and signal processing pipeline.
The system captures IMU data at hardware limits (6.66kHz), transfers it via SPI Batching,
and applies a 6-Bank Active Harmonic Notch Filter on the RPi 4 to mitigate motor vibrations.

- **Node A (Pico 2A)**: SPI Slave. Buffers high-freq samples.
- **Node B (RPi 4)**: SPI Master. Performs Notch Filtering. Publishes ROS 2 topics.

## 2. HARDWARE CONFIGURATION
----------------------------------------------------------------
Reference: `WIRING_DIAGRAM.md`

### [SPI Connection: RPi 4 (Master) <-> Pico (Slave)]
- **Bus Speed**: 10 MHz - 20 MHz
- **SCLK**: RPi4 Pin 23 <-> Pico Pin 24 (GP18)
- **MOSI**: RPi4 Pin 19 <-> Pico Pin 21 (GP16)
- **MISO**: RPi4 Pin 21 <-> Pico Pin 25 (GP19)
- **CS**:   RPi4 Pin 29 <-> Pico Pin 22 (GP17)
- **DRDY**: RPi4 Pin 31 <-> Pico Pin 29 (GP22) (Data Ready Interrupt)

### [Sensors on Pico]
- **IMU (ISM330DHCX)**: SPI1, CS=GP13. 
  * **ODR Setting**: 6.66kHz (High Performance Mode, Reg `0xA0`)
- **Mag (MMC5983MA)**:  SPI1, CS=GP14.
  * **ODR Setting**: 100Hz

## 3. ARCHITECTURE: "Buffered Batch Transfer"
----------------------------------------------------------------
Since the IMU runs at 6.66kHz and the Control Loop at 1kHz, we must batch data.
Note: 6667 Hz / 1000 Hz = 6.667 samples/cycle. 
The batch size will fluctuate between 6 and 7 samples per transaction.

### A. Pico Firmware Logic (Node A)
1. **Global Buffer**:
   - `ImuSample batch_buffer[8]` (Fixed size max buffer)
   - `uint8_t valid_sample_count` (Indicates if 6 or 7 samples are valid)
   - `MagSample current_mag`
2. **IMU Interrupt (6.66kHz)**:
   - Read Accel/Gyro.
   - Store in `batch_buffer`. Increment `valid_sample_count`.
3. **SPI Transaction (1kHz Trigger)**:
   - When CS is asserted, DMA transfers the entire structure.
   - After transfer, reset `valid_sample_count` and pointer for the next cycle.

### B. RPi 4 Driver Logic (Node B)
1. **Loop Rate**: 1000Hz (Strict Real-Time).
2. **SPI Transfer**: Receive the packet.
3. **Signal Processing**: Apply 6-Bank Notch Filter to all `valid_sample_count` samples.
4. **Publish**: Send to `/imu/batch_raw`.

## 4. PROTOCOL DEFINITION
----------------------------------------------------------------
Packet Structure (Packed, Little Endian). Total approx 130 bytes.

### [Header]
  0-1:   Magic (0xAA, 0x55)
  2:     Frame ID (uint8)
  3:     System Flags (Error/Safety)
  4-11:  Timestamp (uint64, microseconds of the *latest* sample)

### [Payload: IMU Batch]
  12:    `valid_sample_count` (uint8) -- Value is typically 6 or 7. Max 8.
  13-108: **IMU Samples Array** (8 slots * 12 bytes)
          * Each slot: Accel_X,Y,Z (int16) + Gyro_X,Y,Z (int16)
          * Populate only up to `valid_sample_count`.

### [Payload: Slow Sensors]
  109-114: Mag X, Y, Z (int16)
  115:     Mag Update Counter (uint8) -- Changed only when Mag updates.

### [Footer]
  116: Checksum (XOR or CRC8)

## 5. SIGNAL PROCESSING: 6-BANK ACTIVE NOTCH FILTER (CRITICAL)
----------------------------------------------------------------
**Location**: RPi 4 Node (Inside Driver or Pre-processor).
**Objective**: Filter specific vibration frequencies of ALL 6 motors individually.

### Logic:
1. Subscribe to ESC Telemetry (RPM of Motors 1-6).
2. Instantiate 6 separate Biquad IIR Notch Filters (F1..F6).
3. For EACH sample in the received batch (0 to `valid_sample_count`-1):
     `Input -> [F1] -> [F2] -> [F3] -> [F4] -> [F5] -> [F6] -> Output`
     *Filter must persist state (history) across batches.*

### !!! SAFETY CONSTRAINTS - PHASE LAG MITIGATION !!!
Cascading 6 filters creates phase lag. To prevent control instability:

**Rule 1: Minimum Frequency Clamp (Idle Protection)**
   - Calculate Target Freq: `F_target = RPM / 60.0`
   - Apply Clamp: `Filter_Freq = MAX( F_target, 80.0 )`
   - **Reasoning**: If motor is at Idle (e.g., 20Hz), DO NOT let the filter drop to 20Hz.
     Keep it parked at 80Hz. Phase lag at 20Hz (Control Band) causes crashes.

**Rule 2: Q-Factor Configuration**
   - Set Q-Factor between **3.0 and 5.0**.
   - A narrow notch is required to minimize phase distortion width.

## 6. IMPLEMENTATION REQUIREMENTS
----------------------------------------------------------------
### [Pico Firmware]
- Use `hardware_spi` and `hardware_dma`.
- IMU ISR must be extremely short to avoid blocking SPI DMA.
- Ensure `valid_sample_count` is thread-safe (atomic access).

### [RPi 4 ROS 2 Node]
- Process Priority: `SCHED_FIFO`, Priority 80+.
- Interface: Use `ImuBatch.msg` containing `sensor_msgs/Imu[]`.
  * Set `header.stamp` for the batch.
  * For individual samples in the array, calculate timestamps by subtracting
    dt (150us) sequentially from the end, or use captured deltas if available.
