/**
 * @file pico_node_full.c
 * @brief Full Pico firmware with 1kHz IMU batching (No FreeRTOS - Superloop)
 * 
 * Architecture:
 * - 1kHz hardware timer for precise IMU sampling
 * - Batch 4 samples → 250Hz publish to /pico/imu_batch
 * - Motor command subscription from /pico/motor_commands
 * - Simple superloop - proven to work with Micro-ROS
 */

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "pico_uart_transports.h"
#include "drivers/mpu6050.h"
#include "drivers/dshot.h"
#include "drivers/telemetry.h"

// --- Configuration ---
#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1
#define BATCH_SIZE 4        // 1kHz / 4 = 250Hz publish rate
#define SAMPLE_PERIOD_US 1000  // 1kHz = 1000us

// PIO Config for Telemetry (DShot uses pio0 sm0-3, pio1 sm0-1 internally)
#define TELEM_PIO pio1
#define TELEM_SM 2

// --- Globals ---
static volatile bool timer_fired = false;
static mpu6050_data_t imu_samples[BATCH_SIZE];
static volatile uint8_t sample_idx = 0;
static volatile bool batch_ready = false;

// Motor command buffer
static float motor_commands[6] = {0};
static volatile uint32_t motor_last_update_ms = 0;

// Telemetry (last known RPMs)
static float esc_rpms[6] = {0};
static uint8_t telem_idx = 0;

// ROS objects
static rcl_publisher_t imu_pub;
static rcl_subscription_t motor_sub;
static std_msgs__msg__Float32MultiArray imu_msg;
static std_msgs__msg__Float32MultiArray motor_msg;
static float imu_data_buffer[6 * BATCH_SIZE];
static float motor_data_buffer[6];

// --- Timer Callback REMOVED ---
// bool timer_callback(repeating_timer_t *rt) {
//     (void)rt;
//     timer_fired = true;
//     return true;  // Keep repeating
// }

// --- Motor Command Callback ---
void motor_callback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg = 
        (const std_msgs__msg__Float32MultiArray *)msgin;
    
    if (msg->data.size == 6) {
        for (int i = 0; i < 6; i++) {
            motor_commands[i] = msg->data.data[i];
        }
        motor_last_update_ms = to_ms_since_boot(get_absolute_time());
    }
}

// --- Main ---
int main() {
    // Initialize stdio for USB CDC
    stdio_init_all();
    
    // LED for status
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);
    
    // I2C Init for MPU6050
    i2c_init(I2C_PORT, 400 * 1000);  // 400kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    
    // MPU6050 Init (no return value check, will loop if read fails)
    mpu6050_init(I2C_PORT);
    
    // DShot Init (6 motors via PIO)
    dshot_init();
    
    // Telemetry Init (ESC RPM feedback)
    telemetry_init(TELEM_PIO, TELEM_SM);
    
    // Wait for USB enumeration
    sleep_ms(2000);
    
    // Setup Micro-ROS transport
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open, pico_serial_transport_close,
        pico_serial_transport_write, pico_serial_transport_read
    );
    
    // Allocator and support
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    // Ping agent until connected (LED blinks while waiting)
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        gpio_put(25, !gpio_get(25));  // Toggle LED
        sleep_ms(200);
    }
    gpio_put(25, 1);  // LED ON = connected
    
    // Initialize support
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        // Error 1: Support Init Failed (1 blink)
        while(1) {
            gpio_put(25, 1); sleep_ms(200);
            gpio_put(25, 0); sleep_ms(1000);
        }
    }
    
    // Create node
    rcl_node_t node;
    ret = rclc_node_init_default(&node, "pico_node", "", &support);
    if (ret != RCL_RET_OK) {
        // Error 2: Node Init Failed (2 blinks)
        while(1) {
            for(int i=0; i<2; i++) { gpio_put(25, 1); sleep_ms(200); gpio_put(25, 0); sleep_ms(200); }
            sleep_ms(1000);
        }
    }
    
    // IMU Publisher (Best Effort for high rate)
    imu_msg.data.capacity = 6 * BATCH_SIZE;
    imu_msg.data.data = imu_data_buffer;
    imu_msg.data.size = 6 * BATCH_SIZE;
    
    ret = rclc_publisher_init_best_effort(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pico/imu_batch"
    );
    if (ret != RCL_RET_OK) {
        // Error 3: Pub Init Failed (3 blinks)
        while(1) {
            for(int i=0; i<3; i++) { gpio_put(25, 1); sleep_ms(200); gpio_put(25, 0); sleep_ms(200); }
            sleep_ms(1000);
        }
    }
    
    // Motor Subscriber
    motor_msg.data.capacity = 6;
    motor_msg.data.data = motor_data_buffer;
    
    ret = rclc_subscription_init_best_effort(
        &motor_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pico/motor_commands"
    );
    if (ret != RCL_RET_OK) {
        // Error 4: Sub Init Failed (4 blinks)
        while(1) {
            for(int i=0; i<4; i++) { gpio_put(25, 1); sleep_ms(200); gpio_put(25, 0); sleep_ms(200); }
            sleep_ms(1000);
        }
    }
    
    // Executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &motor_sub, &motor_msg, 
                                    &motor_callback, ON_NEW_DATA);
    
    // Start 1kHz timer REMOVED - using polling loop
    // static repeating_timer_t timer;
    // add_repeating_timer_us(-SAMPLE_PERIOD_US, timer_callback, NULL, &timer);
    
    // Signal ready with 3 blinks
    for (int i = 0; i < 3; i++) {
        gpio_put(25, 0); sleep_ms(100);
        gpio_put(25, 1); sleep_ms(100);
    }
    
    // --- Main Superloop (Polling) ---
    while (true) {
        // Polling loop ~250Hz
        sleep_ms(4);
        
        // 1. Fill Real Sensor Batch
        // Try to read MPU6050 (if fails, we just skip this sample, or could fill with last known)
        if (mpu6050_read_burst(I2C_PORT, &imu_samples[sample_idx])) {
            sample_idx++;
        }
        
        // Batch full? Publish
        if (sample_idx >= BATCH_SIZE) {
            // Pack samples into message
            for (int i = 0; i < BATCH_SIZE; i++) {
                int base = i * 6;
                imu_data_buffer[base + 0] = (float)imu_samples[i].ax;
                imu_data_buffer[base + 1] = (float)imu_samples[i].ay;
                imu_data_buffer[base + 2] = (float)imu_samples[i].az;
                imu_data_buffer[base + 3] = (float)imu_samples[i].gx;
                imu_data_buffer[base + 4] = (float)imu_samples[i].gy;
                imu_data_buffer[base + 5] = (float)imu_samples[i].gz;
            }
            
            // Publish
            rcl_ret_t rc = rcl_publish(&imu_pub, &imu_msg, NULL);
            sample_idx = 0;
            
            // Heartbeat: Blink every 250 publishes (approx 1Hz)
            static int pub_count = 0;
            if (++pub_count >= 250) {
                gpio_put(25, !gpio_get(25));
                pub_count = 0;
            }
        }
        
        // 5. Motor watchdog
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - motor_last_update_ms > 100) {
            for (int i = 0; i < 6; i++) motor_commands[i] = 0.0f;
        }
        
        // DShot Output
        for (int i = 0; i < 6; i++) dshot_write_throttle(i, motor_commands[i]);
    }
    
    return 0;
}
