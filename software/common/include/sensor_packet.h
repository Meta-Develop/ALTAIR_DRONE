// sensor_packet.h
// ============================================================================
// CRITICAL: This file MUST be IDENTICAL on Pico Firmware and RPi4 ROS 2 Node.
// Any changes here MUST be synchronized to both locations.
// Location Pico: Software/firmware/pico_sensors/sensor_packet.h (symlink)
// Location RPi:  Software/common/include/sensor_packet.h
// ============================================================================
#ifndef SENSOR_PACKET_H
#define SENSOR_PACKET_H

#include <stdint.h>

// Buffer size to hold IMU samples for 1ms at 6.6kHz (approx 7 samples) -> 8 for safety
#define IMU_BATCH_SIZE 8 

struct __attribute__((packed)) ImuSample {
    float accel[3];         // x, y, z (m/s^2)
    float gyro[3];          // x, y, z (rad/s)
    uint64_t timestamp_us;  // Exact acquisition time from Pico hardware timer
};

struct __attribute__((packed)) SensorPacket {
    // --- Header ---
    uint32_t packet_seq;       // Packet sequence number
    
    // --- IMU Batch Data (High Frequency) ---
    uint8_t valid_imu_count;   // Number of valid samples in this packet
    ImuSample imu_samples[IMU_BATCH_SIZE]; 

    // --- Low Frequency Sensors (Snapshot) ---
    float mag[3];              // Magnetometer
    uint8_t mag_seq;           // Increment when updated
    
    float pressure;            // Barometer
    float temperature;
    uint8_t baro_seq;

    float distance_mm;         // ToF
    uint8_t tof_seq;

    // --- Footer ---
    uint32_t checksum;         // Simple XOR or CRC
};

#endif // SENSOR_PACKET_H
