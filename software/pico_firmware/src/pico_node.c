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

// Timer Callback for ESC Telemetry (50Hz)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        // Update ESC message
        xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);
        for (int i=0; i<6; i++) {
            esc_msg.data.data[i] = g_sensor_state.esc_rpm[i];
        }
        xSemaphoreGive(g_sensor_mutex);
        
        // Publish
        rcl_publish(&esc_pub, &esc_msg, NULL);
    }
}

// Queue for IMU samples
QueueHandle_t g_imu_queue;
#define BATCH_SIZE 20 // 20ms batch = 50Hz publish rate

void task_control(void *params) {
    dshot_init();
    telemetry_init(pio1, 2);

    // Debug Blink: LED ON = Entering BNO Init
    gpio_put(25, 1);
    sleep_ms(100); 
    gpio_put(25, 0);
    sleep_ms(100); 
    gpio_put(25, 1);
    
    bool init_ok = bno055_init(i2c0, 0, 1); // Use GP0/GP1 as per debug_report.md
    
    if (!init_ok) {
        // Init Failed: Blink Rapidly 3 times
        for(int i=0; i<3; i++) {
             gpio_put(25, 0); sleep_ms(100);
             gpio_put(25, 1); sleep_ms(100);
        }
    }

    // Debug Blink: LED OFF = BNO Init Success (or finished failing)
    gpio_put(25, 0);

    // Create Queue
    g_imu_queue = xQueueCreate(100, sizeof(bno055_data_t));

    // Precise Timing
    uint64_t period_us = 1000; // 1kHz
    uint64_t next_wake_time = time_us_64();

    static float loop_cnt = 0;
    static float imu_read_cnt = 0;
    static float imu_fail_cnt = 0;
    static float stage = 0;

    while (true) {
        stage = 1.0f; // Start
        loop_cnt += 1.0f;
        
        // Update Start Stats
        xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);
        g_sensor_state.esc_rpm[0] = loop_cnt;
        g_sensor_state.esc_rpm[2] = stage; 
        g_sensor_state.esc_rpm[4] = imu_fail_cnt;
        xSemaphoreGive(g_sensor_mutex);

        // 1. Read Sensors
        bno055_data_t imu_raw = {0};
        bool imu_ok = bno055_read_raw(&imu_raw);
        
        stage = 2.0f; // Post IMU
        xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);
        g_sensor_state.esc_rpm[2] = stage;
        xSemaphoreGive(g_sensor_mutex);

        if (imu_ok) {
            imu_read_cnt += 1.0f;
        } else {
            imu_fail_cnt += 1.0f;
            // Dummy Data Verification Pattern
            // If BNO fails, generate a pattern to prove transmission link works
            imu_raw.accel_x = (int16_t)(loop_cnt); // Changing value
            imu_raw.accel_y = 1234;                // Constant marker
            imu_raw.accel_z = 5678;
            imu_raw.gyro_x = -1;
            imu_raw.gyro_y = -1;
            imu_raw.gyro_z = -1;
        }
        
        // Always queue data (Real or Dummy) to maintain 1kHz stream
        xQueueSend(g_imu_queue, &imu_raw, 0);

        stage = 3.0f; // Post Queue
        xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);
        g_sensor_state.esc_rpm[1] = imu_read_cnt;
        g_sensor_state.esc_rpm[2] = stage;
        g_sensor_state.esc_rpm[3] = (float)uxQueueMessagesWaiting(g_imu_queue);
        g_sensor_state.esc_rpm[4] = imu_fail_cnt;
        xSemaphoreGive(g_sensor_mutex);

        // 2. Watchdog & Motor Output
        stage = 4.0f; // Pre Motor
        xSemaphoreTake(g_motor_mutex, portMAX_DELAY);
        g_sensor_state.esc_rpm[2] = stage; // Debug
        
        TickType_t now_tick = xTaskGetTickCount();
        bool safe = (now_tick - g_motor_state.last_update_time) < pdMS_TO_TICKS(100);
        
        float output[6];
        for(int i=0; i<6; i++) {
            output[i] = safe ? g_motor_state.motors[i] : 0.0f;
        }
        xSemaphoreGive(g_motor_mutex);

        // DISABLE DSHOT FOR ISOLATION TESTING
        // for(int i=0; i<6; i++) {
        //     dshot_write_throttle(i, output[i]);
        // }

        // Precise Loop Timing
        next_wake_time += period_us;
        uint64_t now = time_us_64();
        if (next_wake_time > now) {
            sleep_us(next_wake_time - now);
        }
        
        stage = 5.0f; // Post Sleep
        xSemaphoreTake(g_sensor_mutex, portMAX_DELAY);
        g_sensor_state.esc_rpm[2] = stage;
        xSemaphoreGive(g_sensor_mutex);
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

    rcl_node_t node;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_executor_t executor;
    rcl_timer_t timer;

    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    // Batch Publisher
    rcl_publisher_t batch_pub;
    std_msgs__msg__Float32MultiArray batch_msg;
    rclc_publisher_init_best_effort(
        &batch_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pico/imu_batch"
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
    
    rclc_subscription_init_best_effort(
        &mode_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "/pico/mode"
    );

    // Timer Init (50Hz = 20ms)
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(20),
        timer_callback
    );

    // Executor: 2 Subs + 1 Timer = 3 Handles
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &motor_sub, &motor_msg, &motor_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &mode_sub, &mode_msg, &mode_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    // Static Buffers
    static float motor_data_buffer[6];
    static float esc_data_buffer[6];
    static float batch_data_buffer[6 * BATCH_SIZE];

    // Initialize messages
    motor_msg.data.capacity = 6;
    motor_msg.data.data = motor_data_buffer;
    motor_msg.data.size = 0;

    esc_msg.data.capacity = 6;
    esc_msg.data.data = esc_data_buffer;
    esc_msg.data.size = 6;

    // Batch Message Init
    batch_msg.data.capacity = 6 * BATCH_SIZE; 
    batch_msg.data.data = batch_data_buffer;
    batch_msg.data.size = 0;

    bno055_data_t batch_buffer[BATCH_SIZE];

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        
        // Check Queue
        if (uxQueueMessagesWaiting(g_imu_queue) >= BATCH_SIZE) {
            // Pop batch
            for (int i=0; i<BATCH_SIZE; i++) {
                xQueueReceive(g_imu_queue, &batch_buffer[i], 0);
            }

            // Pack into message
            // Order: ax, ay, az, gx, gy, gz for each sample
            for (int i=0; i<BATCH_SIZE; i++) {
                int base = i * 6;
                batch_msg.data.data[base + 0] = (float)batch_buffer[i].accel_x;
                batch_msg.data.data[base + 1] = (float)batch_buffer[i].accel_y;
                batch_msg.data.data[base + 2] = (float)batch_buffer[i].accel_z;
                batch_msg.data.data[base + 3] = (float)batch_buffer[i].gyro_x;
                batch_msg.data.data[base + 4] = (float)batch_buffer[i].gyro_y;
                batch_msg.data.data[base + 5] = (float)batch_buffer[i].gyro_z;
            }
            batch_msg.data.size = 6 * BATCH_SIZE;

            rcl_publish(&batch_pub, &batch_msg, NULL);
        }

        // Heartbeat
        static int heartbeat_count = 0;
        if (++heartbeat_count >= 50) {
            static bool led_state = false;
            led_state = !led_state;
            gpio_put(25, led_state);
            heartbeat_count = 0;
        }

        // Yield to prevent starvation
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void pico_node_init(void) {
    g_motor_mutex = xSemaphoreCreateMutex();
    g_sensor_mutex = xSemaphoreCreateMutex();
    
    TaskHandle_t control_handle;
    // Stack increased to 2048
    xTaskCreate(task_control, "Control", 2048, NULL, tskIDLE_PRIORITY + 1, &control_handle);
    
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
