/**
 * @file pico_node_simple.c
 * @brief Simplified Micro-ROS node without FreeRTOS for communication testing.
 * 
 * This firmware verifies basic Micro-ROS communication over USB CDC.
 * It does NOT use FreeRTOS to avoid complex build dependencies.
 */

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

// Simple counter publisher
static rcl_publisher_t publisher;
static std_msgs__msg__Int32 msg;

int main() {
    // Initialize stdio for USB CDC
    stdio_init_all();
    
    // LED for status indication
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);
    
    // Wait for USB enumeration
    sleep_ms(2000);
    
    // Setup Micro-ROS transport
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
    
    // Allocator and support
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    // Ping agent until connected
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        gpio_put(25, !gpio_get(25)); // Blink while waiting
        sleep_ms(200);
    }
    gpio_put(25, 1); // LED ON = connected
    
    // Initialize support
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        gpio_put(25, 0);
        while(1) { sleep_ms(100); } // Error halt
    }
    
    // Create node
    rcl_node_t node;
    ret = rclc_node_init_default(&node, "pico_test_node", "", &support);
    if (ret != RCL_RET_OK) {
        gpio_put(25, 0);
        while(1) { sleep_ms(100); }
    }
    
    // Create publisher
    ret = rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/pico/counter"
    );
    if (ret != RCL_RET_OK) {
        gpio_put(25, 0);
        while(1) { sleep_ms(100); }
    }
    
    // Main loop - publish counter every second
    msg.data = 0;
    while (true) {
        ret = rcl_publish(&publisher, &msg, NULL);
        (void)ret; // Ignore for simplicity
        
        msg.data++;
        gpio_put(25, msg.data % 2); // Toggle LED each publish
        
        sleep_ms(1000);
    }
    
    return 0;
}
