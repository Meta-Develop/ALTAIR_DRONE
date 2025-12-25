# Pico SPI Debugging Walkthrough

## 1. Problem Summary
Initial Issue: SPI communication failure, confused device roles.
Resolution: Corrected physical wiring/device assignment.
New Issue: **Sensor Hard Failure**.

## 2. Final System Status
| Bus | Use Case | Connected Device | Status | Protocol Check | Data Content |
|---|---|---|---|---|---|
| **SPI0** | Sensors | Pico 2A (Sensors) | **OK** | `0xAA55` Received | **Header OK, Count=0 (FAIL)** |
| **SPI1** | Actuators | Pico 2B (Actuators) | **OK** | `0xCAFE` Received | N/A |

## 3. Sensor Diagnostics
**Symptoms:**
- **SPI Link Valid**: RPi4 receives packets from Pico.
- **Pico FW Valid**: RP2350 target, SPI Pins Corrected (`GPIO_FUNC_SPI`). Code runs.
- **Sensor Data Missing**: Both SPI (IMU/Mag) and I2C (Baro) fail initialization. `Count=0`.

**Failed Attempts:**
1.  **Reflash to RP2350**: No Change.
2.  **Fix SPI Pin Function**: Corrected `GPIO_FUNC_MATCH_SPI` typo. No Change.
3.  **Capture Init Logs**: Shows `WHO_AM_I: 0x00` (Device not responding).

**Conclusion:**
There is a **Hardware Failure** on the Sensor Board side.
Possible causes:
- **Dead Sensors**: MEMS chips damaged (heat/static).
- **Data Line Disconnect**: Even if 3V3 is good, MISO/MOSI/SCL/SDA might have cold solder joints or broken traces.

## 4. Recommendation
**Replace Component**: The SparkFun 9DoF Sensor Board (and Baro) needs replacement or deep hardware inspection (Oscilloscope).
**Software is Verified**: The current `pico_sensors` firmware is correct and ready for working hardware.
