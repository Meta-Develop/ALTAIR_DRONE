/**
 * @file pico_node_imu.c
 * @brief Superloop Firmware with IMU (MPU6050) + Micro-ROS
 * 
 * Features:
 * - MPU6050 Burst Read (1kHz)
 * - /pico/imu_batch: Float32MultiArray (4 samples = 250Hz Packet Rate)
 * - /pico/imu_raw: Float32MultiArray (Decimated to 10Hz for monitoring)
 * 
 * Architecture: Superloop (No FreeRTOS)
 */

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "drivers/mpu6050.h"

// --- Config ---
#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1
#define BATCH_SIZE 4 // 1kHz / 4 = 250Hz
#define RAW_DECIMATION 100 // 1kHz / 100 = 10Hz

// --- Transport Externs ---
extern bool pico_serial_transport_open(struct uxrCustomTransport * transport);
extern bool pico_serial_transport_close(struct uxrCustomTransport * transport);
extern size_t pico_serial_transport_write(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, uint8_t *errcode);
extern size_t pico_serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode);

// --- Globals ---
rcl_publisher_t batch_pub;
rcl_publisher_t raw_pub;
std_msgs__msg__Float32MultiArray batch_msg;
std_msgs__msg__Float32MultiArray raw_msg;

// Buffer for Batch (6 axes * 4 samples)
float batch_data[6 * BATCH_SIZE];
// Buffer for Raw (6 axes)
float raw_data[6];

// --- Main ---
int main() {
    // 1. Hardware Init
    // stdio_init_all(); // Called by transport? Safest to call if using USB printf, but we use Micro-ROS.
    
    // LED
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);

    // I2C & MPU6050
    i2c_init(I2C_PORT, 400 * 1000); // 400kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    
    sleep_ms(100);
    mpu6050_init(I2C_PORT);
    
    // 2. Micro-ROS Transport
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open, pico_serial_transport_close,
        pico_serial_transport_write, pico_serial_transport_read
    );

    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Ping Loop
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        gpio_xor_mask(1 << 25);
        sleep_ms(200);
    }
    gpio_put(25, 1); // Solid ON

    // 3. Node & Pub Init
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    // Batch Publisher (250Hz)
    // Msg Init
    batch_msg.data.capacity = 6 * BATCH_SIZE;
    batch_msg.data.data = batch_data;
    batch_msg.data.size = 6 * BATCH_SIZE;
    
    rclc_publisher_init_best_effort(
        &batch_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pico/imu_batch"
    );

    // Raw Publisher (10Hz Monitor)
    raw_msg.data.capacity = 6;
    raw_msg.data.data = raw_data;
    raw_msg.data.size = 6;

    rclc_publisher_init_best_effort(
        &raw_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pico/imu_raw"
    );

    // 4. Main Loop
    int batch_idx = 0;
    int raw_decimation_cnt = 0;
    mpu6050_data_t imu_data = {0}; // Struct for MPU6050 data
    
    uint64_t last_read_time = time_us_64();
    const uint64_t READ_INTERVAL_US = 1000; // 1kHz

    while (true) {
        uint64_t now = time_us_64();
        
        // 1kHz Loop Logic
        if (now - last_read_time >= READ_INTERVAL_US) {
            last_read_time += READ_INTERVAL_US;

            // Try to read MPU - if fails, data stays as zeros
            mpu6050_read_burst(I2C_PORT, &imu_data);
            
            // ALWAYS add to batch (even if zeros)
            batch_data[batch_idx * 6 + 0] = (float)imu_data.ax;
            batch_data[batch_idx * 6 + 1] = (float)imu_data.ay;
            batch_data[batch_idx * 6 + 2] = (float)imu_data.az;
            batch_data[batch_idx * 6 + 3] = (float)imu_data.gx;
            batch_data[batch_idx * 6 + 4] = (float)imu_data.gy;
            batch_data[batch_idx * 6 + 5] = (float)imu_data.gz;
            
            batch_idx++;

            // ALWAYS publish batch when full
            if (batch_idx >= BATCH_SIZE) {
                rcl_publish(&batch_pub, &batch_msg, NULL);
                batch_idx = 0;
                gpio_xor_mask(1 << 25); // Toggle LED on publish
            }

            // Publish Raw (Decimated) - ALWAYS
            raw_decimation_cnt++;
            if (raw_decimation_cnt >= RAW_DECIMATION) {
                raw_data[0] = (float)imu_data.ax;
                raw_data[1] = (float)imu_data.ay;
                raw_data[2] = (float)imu_data.az;
                raw_data[3] = (float)imu_data.gx;
                raw_data[4] = (float)imu_data.gy;
                raw_data[5] = (float)imu_data.gz;
                rcl_publish(&raw_pub, &raw_msg, NULL);
                raw_decimation_cnt = 0;
            }
        }
    }
}
