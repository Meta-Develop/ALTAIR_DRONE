#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transport.h"
#include "drivers/ism330dhcx.h" 

// --- PIN DEFINITIONS (Hardware SPI1 for Sensor) ---
#define SENSOR_SPI_PORT spi1
#define PIN_SENS_RX   12 // MISO
#define PIN_SENS_CS   13 // CS
#define PIN_SENS_SCK  14 // SCK
#define PIN_SENS_TX   15 // MOSI
#define PIN_LED       25

// --- GLOBAL ROS OBJECTS ---
rcl_publisher_t publisher;
sensor_msgs__msg__Imu msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Status Flags
bool sensor_ready = false;

// --- ERROR HANDLING ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ return; }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// --- SENSOR SETUP ---
void init_sensors() {
    // Hardware SPI Init for ISM330
    spi_init(SENSOR_SPI_PORT, 2000 * 1000); 
    gpio_set_function(PIN_SENS_RX,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_SENS_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SENS_TX,  GPIO_FUNC_SPI);
    
    // Manual CS
    gpio_init(PIN_SENS_CS);
    gpio_set_dir(PIN_SENS_CS, GPIO_OUT);
    gpio_put(PIN_SENS_CS, 1);

    // Init Driver
    for(int i=0; i<5; i++) {
        if(ism330_init(SENSOR_SPI_PORT, PIN_SENS_CS)) {
            sensor_ready = true;
            break;
        }
        sleep_ms(100);
    }
}

// --- TIME SHIM (Linker Fix for Micro-ROS) ---
#include <time.h>
int clock_gettime(clockid_t clk_id, struct timespec *tp) {
    (void) clk_id;
    uint64_t now_us = time_us_64();
    tp->tv_sec = now_us / 1000000;
    tp->tv_nsec = (now_us % 1000000) * 1000;
    return 0;
}

// --- TIMER CALLBACK ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void) last_call_time;
    if (timer != NULL) {
        // 1. Read Sensor
        ism330_data_t raw_data;
        if(sensor_ready) {
             ism330_read_data(SENSOR_SPI_PORT, PIN_SENS_CS, &raw_data);
             
             // 2. Populate Message
             msg.linear_acceleration.x = raw_data.accel[0]; // m/s^2
             msg.linear_acceleration.y = raw_data.accel[1];
             msg.linear_acceleration.z = raw_data.accel[2];
             
             msg.angular_velocity.x = raw_data.gyro[0]; // rad/s
             msg.angular_velocity.y = raw_data.gyro[1];
             msg.angular_velocity.z = raw_data.gyro[2];
        } else {
             // Error State: Send Zeros or Counter
             msg.linear_acceleration.z = 9.81; 
        }

        // 3. Publish
        // No timestamp? Micro-ROS often relies on Agent time or simple counter.
        // We can add header if needed, but keeping it simple.
        msg.header.frame_id.data = "pico_link";
        msg.header.frame_id.size = strlen(msg.header.frame_id.data);
        
        rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);
        
        // Blink on Publish
        gpio_put(PIN_LED, !gpio_get(PIN_LED));
    }
}

int main() {
// 1. Hardware Init
    stdio_init_all(); 
    
    // LED Init
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    
    // NO USB WAIT - We assume UART is always ready via Debug Probe
    printf("UART READY\r\n");

    // Sensor Init
    printf("SENSORS INIT...\r\n");
    init_sensors();
    printf("SENSORS DONE\r\n");
    
    // 2. Micro-ROS Transport Init (UART)
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_uart_transport_open,
		pico_uart_transport_close,
		pico_uart_transport_write,
		pico_uart_transport_read
	);

    // 3. Main Loop (Re-connection Logic)
    while (true) {
        // Ping Agent (Blink fast while searching)
        gpio_put(PIN_LED, 1);
        if(rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            gpio_put(PIN_LED, 0);
            sleep_ms(100); 
            continue;
        }
        gpio_put(PIN_LED, 0); // Connected


        // Allocator
        allocator = rcl_get_default_allocator();

        // Support
        RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

        // Node
        RCCHECK(rclc_node_init_default(&node, "pico_sensors", "", &support));

        // Publisher
        RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "/pico/imu"));

        // Timer (100Hz = 10ms)
        RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(10),
            timer_callback));

        // Executor
        RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
        RCCHECK(rclc_executor_add_timer(&executor, &timer));

        // Spin
        while (true) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            // Check connection?
            // If ping fails...
            // break;
        }

        // Clean up
        rclc_executor_fini(&executor);
        rclc_publisher_fini(&publisher);
        rclc_timer_fini(&timer);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
    }
    return 0;
}
