#include "pico_node.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "drivers/dshot.h"
#include "drivers/telemetry.h"
#include "drivers/mpu6050.h"

// --- Config ---
#define I2C_PORT i2c0
#define SDA_PIN 0
#define SCL_PIN 1

// PIO Config for Telemetry (DShot uses pio0 sm0-3, pio1 sm0-1 internally)
#define TELEM_PIO pio1
#define TELEM_SM 2

// --- Globals ---
motor_state_t g_motor_state;
QueueHandle_t g_sensor_queue;
SemaphoreHandle_t g_motor_mutex;

// --- FreeRTOS Hooks ---
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask; (void)pcTaskName;
    while(1) { gpio_put(25, 1); sleep_ms(50); gpio_put(25, 0); sleep_ms(50); }
}

void vApplicationMallocFailedHook(void) {
    while(1) { gpio_put(25, 1); sleep_ms(200); gpio_put(25, 0); sleep_ms(200); }
}

void vApplicationTickHook(void) {}
void vApplicationIdleHook(void) {}

// --- Transport ---
extern bool pico_serial_transport_open(struct uxrCustomTransport * transport);
extern bool pico_serial_transport_close(struct uxrCustomTransport * transport);
extern size_t pico_serial_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
extern size_t pico_serial_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

// --- Time Helpers ---
int64_t uxr_millis(void) { return to_ms_since_boot(get_absolute_time()); }
int64_t uxr_nanos(void) { return to_us_since_boot(get_absolute_time()) * 1000; }

// --- Tasks ---

// Core 1: Sensor & Control Task
void task_sensor(void *params) {
    (void)params;
    
    // I2C Init
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    
    mpu6050_init(I2C_PORT);
    
    // DShot Init
    dshot_init();
    
    // Telemetry Init
    telemetry_init(TELEM_PIO, TELEM_SM);
    
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1kHz
    xLastWakeTime = xTaskGetTickCount();
    
    motor_state_t local_motor_state;
    // Init local state
    for(int i=0; i<6; i++) local_motor_state.motors[i] = 0.0f;
    local_motor_state.last_update_time_ms = 0;

    int telem_idx = 0;
    float telem_rpms[6] = {0};
    sensor_packet_t packet;

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // 1. Read IMU (Burst)
        packet.timestamp_us = to_us_since_boot(get_absolute_time());
        if (!mpu6050_read_burst(I2C_PORT, &packet.imu)) {
            // Error handling could be added here
        }
        
        // Update Telemetry into packet (using last knowns)
        for(int i=0; i<6; i++) packet.esc_rpm[i] = telem_rpms[i];
        
        // Overwrite queue to keep latest data
        xQueueOverwrite(g_sensor_queue, &packet);
        
        // 3. Motor Output (Watchdog check)
        if (xSemaphoreTake(g_motor_mutex, 0) == pdTRUE) { // Non-blocking
            local_motor_state = g_motor_state;
            xSemaphoreGive(g_motor_mutex);
        }
        
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - local_motor_state.last_update_time_ms > 100) {
            // Watchdog: Stop
            for(int i=0; i<6; i++) dshot_write_throttle(i, 0.0f);
        } else {
            // Write Throttles
            for(int i=0; i<6; i++) dshot_write_throttle(i, local_motor_state.motors[i]);
        }
    }
}

// Forward declare callback
void motor_cb(const void * msgin);

// Core 0: Comms Task (Micro-ROS)
#define BATCH_SIZE 4 // 1kHz / 4 = 250Hz Packet Rate
void task_ros(void *params) {
    (void)params;

    // Transport Init
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open, pico_serial_transport_close,
        pico_serial_transport_write, pico_serial_transport_read
    );

    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Ping
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) { vTaskDelay(pdMS_TO_TICKS(100)); }
    gpio_put(25, 1);

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    // IMU Pub
    rcl_publisher_t imu_pub;
    std_msgs__msg__Float32MultiArray imu_msg;
    // 6 values * BATCH_SIZE floats
    float imu_data[6 * BATCH_SIZE]; 
    imu_msg.data.capacity = 6 * BATCH_SIZE;
    imu_msg.data.data = imu_data;
    imu_msg.data.size = 6 * BATCH_SIZE;
    
    rclc_publisher_init_best_effort(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pico/imu_batch"
    );

    // Motor Sub
    rcl_subscription_t motor_sub;
    std_msgs__msg__Float32MultiArray motor_msg;
    float motor_data_buffer[6];
    motor_msg.data.capacity = 6;
    motor_msg.data.data = motor_data_buffer;
    
    rclc_subscription_init_best_effort(
        &motor_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pico/motor_commands"
    );

    // Executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &motor_sub, &motor_msg, &motor_cb, ON_NEW_DATA);

    sensor_packet_t batch_buffer[BATCH_SIZE];
    int batch_idx = 0;
    sensor_packet_t packet;

    while (true) {
        // 1. Process Incoming (Motors)
        rclc_executor_spin_some(&executor, 1000 * 10); // Wait up to 10us? No, unit is nanos. 10000ns = 10us.
        
        // 2. Process Outgoing (IMU)
        // Drain Queue
        while (xQueueReceive(g_sensor_queue, &packet, 0) == pdTRUE) {
            batch_buffer[batch_idx++] = packet;
            
            if (batch_idx >= BATCH_SIZE) {
                // Flatten
                for(int i=0; i<BATCH_SIZE; i++) {
                    int base = i*6;
                    imu_data[base+0] = (float)batch_buffer[i].imu.ax;
                    imu_data[base+1] = (float)batch_buffer[i].imu.ay;
                    imu_data[base+2] = (float)batch_buffer[i].imu.az;
                    imu_data[base+3] = (float)batch_buffer[i].imu.gx;
                    imu_data[base+4] = (float)batch_buffer[i].imu.gy;
                    imu_data[base+5] = (float)batch_buffer[i].imu.gz;
                    // Note: This is RAW INT16 cast to float. Unpacker node must scale.
                }
                rcl_publish(&imu_pub, &imu_msg, NULL);
                batch_idx = 0;
            }
        }
    }
}

// Subscription Callback
void motor_cb(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    if (msg->data.size == 6) {
        if (xSemaphoreTake(g_motor_mutex, 10) == pdTRUE) {
            for(int i=0; i<6; i++) {
                g_motor_state.motors[i] = msg->data.data[i];
            }
            g_motor_state.last_update_time_ms = to_ms_since_boot(get_absolute_time());
            xSemaphoreGive(g_motor_mutex);
        }
    }
}

// Initialization
void pico_node_init(void) {
    g_sensor_queue = xQueueCreate(10, sizeof(sensor_packet_t)); 
    g_motor_mutex = xSemaphoreCreateMutex();
    
    // Core 0: Comm
    xTaskCreate(task_ros, "ROS", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    
    // Core 1: Sensor (Pinned if possible)
    TaskHandle_t hSensor;
    xTaskCreate(task_sensor, "Sensor", 2048, NULL, tskIDLE_PRIORITY + 4, &hSensor);
    vTaskCoreAffinitySet(hSensor, (1 << 1)); // Core 1
}
