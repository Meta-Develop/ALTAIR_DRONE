#ifndef BATCH_PROTOCOL_H
#define BATCH_PROTOCOL_H

#include <stdint.h>

// Protocol Constants
#define BATCH_MAGIC_0    0xAA
#define BATCH_MAGIC_1    0x55
#define MAX_BATCH_SIZE   8
#define TOTAL_PACKET_SIZE 136  // Adjusted for alignment (see struct)

// IMU Sample (12 bytes)
// Little Endian: ax, ay, az, gx, gy, gz
typedef struct __attribute__((packed)) {
    int16_t accel[3]; // X, Y, Z
    int16_t gyro[3];  // X, Y, Z
} ImuSample;

// Batch Packet Structure
// Total Size matches DMA transfer size
typedef struct __attribute__((packed)) {
    // Header (12 bytes)
    uint8_t magic[2];          // 0xAA, 0x55
    uint8_t frame_id;          // Incrementing ID
    uint8_t system_flags;      // Error flags, safety status
    uint64_t timestamp_us;     // Timestamp of the LATEST sample in batch

    // IMU Batch Payload (97 bytes)
    uint8_t valid_sample_count;            // Number of valid samples in this batch (6 or 7)
    ImuSample samples[MAX_BATCH_SIZE];     // 8 * 12 = 96 bytes

    // Slow Sensors Payload (sensors read at lower rates) (7 bytes)
    int16_t mag[3];            // X, Y, Z
    uint8_t mag_count;         // Increments when mag data is fresh

    int32_t pressure_raw;      // Barometer raw data (4 bytes)
    uint16_t tof_mm;           // ToF distance in mm (2 bytes)

    // Padding/Footer (To align to 32-bit or fixed size)
    // Current total: 12 + 1 + 96 + 6 + 1 + 4 + 2 = 122 bytes.
    // Let's pad to 128 bytes (power of 2 is nice for DMA) or keep compact.
    // The user spec said "Total approx 130 bytes".
    // Let's add a checksum and pad to 128 bytes for 32-bit alignment friendliness.
    
    // Checksum
    uint8_t checksum;          // XOR of all previous bytes
    
    // Padding to reach 128 bytes (128 - 123 = 5 bytes padding)
    uint8_t padding[5];

} BatchPacket; // Total Size: 128 bytes

// Check alignment
// 122 data + 1 checksum + 5 padding = 128 bytes.

#endif // BATCH_PROTOCOL_H
