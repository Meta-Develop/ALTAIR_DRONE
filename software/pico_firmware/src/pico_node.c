#include "pico_node.h"
#include "pico/stdlib.h"
#include <stdlib.h>
#include "drivers/dshot.h"
#include "drivers/bno055.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transports.h" // Standard Pico micro-ROS transport

// Global State
motor_state_t g_motor_state;
SemaphoreHandle_t g_motor_mutex;

sensor_state_t g_sensor_state;
SemaphoreHandle_t g_sensor_mutex;

// Micro-ROS entities
rcl_publisher_t imu_pub;
rcl_publisher_t esc_pub;
rcl_subscription_t motor_sub;

std_msgs__msg__Float32MultiArray motor_msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32MultiArray esc_msg;

void motor_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    
    if (msg->data.size == 6) {
        xSemaphoreTake(g_motor_mutex, portMAX_DELAY);
        for (int i = 0; i < 6; i++) {
            g_motor_state.motors[i] = msg->data.data[i];
        }
        g_motor_state.last_update_time = xTaskGetTickCount();
        xSemaphoreGive(g_motor_mutex);
    }
}

void task_control(void *params) {
    dshot_init();
    bno055_init(i2c0, 0, 1); // I2C0, SDA=GP0, SCL=GP1

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500Hz approx

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, 2); // 2ms period

        // 1. Read Sensors
        bno055_data_t imu_raw = {0};
        bool imu_ok = bno055_read_raw(&imu_raw);
        
        // (Telemetry read placeholder)
        float rpm_values[6] = {0}; 
        // TODO: Implement PIO UART RX for telemetry

        // Update Shared Sensor State
        xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);
        if (imu_ok) {
            g_sensor_state.imu = imu_raw;
        }
        for(int i=0; i<6; i++) g_sensor_state.esc_rpm[i] = rpm_values[i];
        xSemaphoreGive(g_sensor_mutex);

        // 2. Watchdog & Motor Output
        xSemaphoreTake(g_motor_mutex, portMAX_DELAY);
        TickType_t now = xTaskGetTickCount();
        bool safe = (now - g_motor_state.last_update_time) < pdMS_TO_TICKS(100);
        
        float output[6];
        for(int i=0; i<6; i++) {
            output[i] = safe ? g_motor_state.motors[i] : 0.0f;
        }
        xSemaphoreGive(g_motor_mutex);

        for(int i=0; i<6; i++) {
            dshot_write_throttle(i, output[i]);
        }
    }
}

void task_microros(void *params) {
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;

    // Wait for agent
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_publisher_init_best_effort(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/pico/imu_raw"
    );

    rclc_publisher_init_best_effort(
        &esc_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pico/esc_telemetry"
    );

    rclc_subscription_init_best_effort(
        &motor_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pico/motor_commands"
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &motor_sub, &motor_msg, &motor_callback, ON_NEW_DATA);

    // Initialize messages
    motor_msg.data.capacity = 6;
    motor_msg.data.data = (float*) malloc(6 * sizeof(float));
    motor_msg.data.size = 0;

    esc_msg.data.capacity = 6;
    esc_msg.data.data = (float*) malloc(6 * sizeof(float));
    esc_msg.data.size = 6;

    // Properly initialize IMU message and its strings
    sensor_msgs__msg__Imu__init(&imu_msg);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        
        xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);
        bno055_data_t imu_data = g_sensor_state.imu;
        for (int i=0; i<6; i++) {
            esc_msg.data.data[i] = g_sensor_state.esc_rpm[i];
        }
        xSemaphoreGive(g_sensor_mutex);

        // Populate IMU msg
        imu_msg.linear_acceleration.x = imu_data.accel_x; // Note: Scaling omitted for raw
        imu_msg.linear_acceleration.y = imu_data.accel_y;
        imu_msg.linear_acceleration.z = imu_data.accel_z;
        imu_msg.angular_velocity.x = imu_data.gyro_x;
        imu_msg.angular_velocity.y = imu_data.gyro_y;
        imu_msg.angular_velocity.z = imu_data.gyro_z;
        
        rcl_publish(&imu_pub, &imu_msg, NULL);
        rcl_publish(&esc_pub, &esc_msg, NULL);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void pico_node_init(void) {
    g_motor_mutex = xSemaphoreCreateMutex();
    g_sensor_mutex = xSemaphoreCreateMutex();
    
    TaskHandle_t control_handle;
    xTaskCreate(task_control, "Control", 1024, NULL, tskIDLE_PRIORITY + 2, &control_handle);
    
    // Set Affinity to Core 1 (Mask 2 -> bit 1 set)
    vTaskCoreAffinitySet(control_handle, (1 << 1));

    xTaskCreate(task_microros, "MicroROS", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
}
