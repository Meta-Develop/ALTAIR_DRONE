#include "pico_node.h"
#include "pico/stdlib.h"
#include <stdlib.h>
#include "drivers/dshot.h"
#include "drivers/bno055.h"
#include "drivers/telemetry.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

// Transport Prototypes
bool pico_serial_transport_open(struct uxrCustomTransport * transport);
bool pico_serial_transport_close(struct uxrCustomTransport * transport);
size_t pico_serial_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t pico_serial_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

// Global State
motor_state_t g_motor_state;
SemaphoreHandle_t g_motor_mutex;

sensor_state_t g_sensor_state;
SemaphoreHandle_t g_sensor_mutex;

// 0 = Test Mode (No Telemetry), 1 = Actual Mode (Telemetry)
volatile int g_system_mode = 0; 

// Micro-ROS entities
rcl_publisher_t imu_pub;
rcl_publisher_t esc_pub;
rcl_subscription_t motor_sub;
rcl_subscription_t mode_sub;

std_msgs__msg__Float32MultiArray motor_msg;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32MultiArray esc_msg;
std_msgs__msg__Int8 mode_msg;

void mode_callback(const void * msgin) {
    const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;
    g_system_mode = msg->data;
    // Optional: Blink LED to indicate mode change?
}

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
    // Initialize Telemetry (PIO 0, SM 2 - DShot uses SM 0,1 if needed, or 0-3?)
    // DShot uses 1 SM per motor? No, dshot.c uses 1 SM per motor?
    // dshot.c: "PIO current_pio = (i < 4) ? pio0 : pio1;"
    // Motors 0-3 use pio0 SM 0-3.
    // Motors 4-5 use pio1 SM 0-1.
    // So pio0 is FULL.
    // We must use pio1 for telemetry.
    // pio1 has SM 2,3 free.
    telemetry_init(pio1, 2);

    // Debug: Turn LED ON before I2C Init
    gpio_put(25, 1);
    bno055_init(i2c0, 0, 1); // I2C0, SDA=GP0, SCL=GP1
    // Debug: Turn LED OFF after I2C Init (Success)
    gpio_put(25, 0);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2); // 500Hz approx

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, 2); // 2ms period

        // 1. Read Sensors
        bno055_data_t imu_raw = {0};
        bool imu_ok = bno055_read_raw(&imu_raw);
        
        // Round-Robin Telemetry
        static int telem_idx = 0;
        float rpm = 0.0f;
        
        // Only read telemetry in Actual Mode (1)
        if (g_system_mode == 1) {
             rpm = telemetry_read_rpm(pio1, 2, telem_idx);
        }
        
        // Store
        xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);
        if (imu_ok) {
            g_sensor_state.imu = imu_raw;
        }
        g_sensor_state.esc_rpm[telem_idx] = rpm;
        xSemaphoreGive(g_sensor_mutex);

        // Advance index
        telem_idx = (telem_idx + 1) % 6;

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
    
    // Mode Subscription
    rclc_subscription_init_best_effort(
        &mode_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "/pico/mode"
    );

    rclc_executor_init(&executor, &support.context, 2, &allocator); // Increased handles to 2
    rclc_executor_add_subscription(&executor, &motor_sub, &motor_msg, &motor_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &mode_sub, &mode_msg, &mode_callback, ON_NEW_DATA);

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
        imu_msg.linear_acceleration.x = (double)imu_data.accel_x;
        imu_msg.linear_acceleration.y = (double)imu_data.accel_y;
        imu_msg.linear_acceleration.z = (double)imu_data.accel_z;
        imu_msg.angular_velocity.x = (double)imu_data.gyro_x;
        imu_msg.angular_velocity.y = (double)imu_data.gyro_y;
        imu_msg.angular_velocity.z = (double)imu_data.gyro_z;
        
        rcl_ret_t ret = rcl_publish(&imu_pub, &imu_msg, NULL);
        (void)ret;
        ret = rcl_publish(&esc_pub, &esc_msg, NULL);
        (void)ret;

        // Heartbeat Blink (Toggle every cycle approx 10ms -> too fast? Make it every 100 cycles)
        static int heartbeat_count = 0;
        if (++heartbeat_count >= 50) {
            static bool led_state = false;
            led_state = !led_state;
            gpio_put(25, led_state);
            heartbeat_count = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void pico_node_init(void) {
    g_motor_mutex = xSemaphoreCreateMutex();
    g_sensor_mutex = xSemaphoreCreateMutex();
    
    TaskHandle_t control_handle;
    // Priority Swap: MicroROS higher than Control to prevent starvation if I2C hangs
    xTaskCreate(task_control, "Control", 1024, NULL, tskIDLE_PRIORITY + 1, &control_handle);
    
    // Set Affinity to Core 1 (Mask 2 -> bit 1 set)
    #if configUSE_CORE_AFFINITY
    vTaskCoreAffinitySet(control_handle, (1 << 1));
    #endif

    xTaskCreate(task_microros, "MicroROS", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
}

// --- Hooks & Compatibility ---

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    panic("Stack Overflow: %s\n", pcTaskName);
}

#include <time.h>
#include <sys/time.h>

int clock_gettime(clockid_t clock_id, struct timespec *tp) {
    (void)clock_id;
    uint64_t now_us = time_us_64();
    tp->tv_sec = now_us / 1000000;
    tp->tv_nsec = (now_us % 1000000) * 1000;
    return 0;
}
