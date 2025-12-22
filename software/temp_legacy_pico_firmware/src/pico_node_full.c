/**
 * @file pico_node_full.c
 * @brief Simple Int32 Publisher Test
 * 
 * Minimal test to verify Micro-ROS publishing works.
 * This is the known-working pattern from pico_node_simple.
 */

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"

// SDK transport externs
extern bool pico_serial_transport_open(struct uxrCustomTransport * transport);
extern bool pico_serial_transport_close(struct uxrCustomTransport * transport);
extern size_t pico_serial_transport_write(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, uint8_t *errcode);
extern size_t pico_serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode);

// ROS objects
static rcl_publisher_t publisher;
static std_msgs__msg__Int32 msg;

int main() {
    // Note: SDK transport calls stdio_init_all() internally
    
    // LED Init
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 0);
    
    sleep_ms(2000);
    
    // Micro-ROS Transport
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open, pico_serial_transport_close,
        pico_serial_transport_write, pico_serial_transport_read
    );
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    // Ping agent (blink while waiting)
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        gpio_put(25, !gpio_get(25));
        sleep_ms(200);
    }
    gpio_put(25, 1); // Connected - solid ON
    
    // Init support
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        while(1) { gpio_put(25, 0); sleep_ms(100); gpio_put(25, 1); sleep_ms(100); }
    }
    
    // Create node
    rcl_node_t node;
    ret = rclc_node_init_default(&node, "pico_node", "", &support);
    if (ret != RCL_RET_OK) {
        while(1) { for(int i=0;i<2;i++){gpio_put(25,0);sleep_ms(100);gpio_put(25,1);sleep_ms(100);} sleep_ms(500); }
    }
    
    // Create publisher - DEFAULT QoS (reliable, like working simple test)
    ret = rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/pico/counter"
    );
    if (ret != RCL_RET_OK) {
        while(1) { for(int i=0;i<3;i++){gpio_put(25,0);sleep_ms(100);gpio_put(25,1);sleep_ms(100);} sleep_ms(500); }
    }
    
    // Main loop - publish counter every second
    msg.data = 0;
    while (true) {
        // Note: No tud_task() needed - SDK transport handles USB via putchar/getchar
        
        ret = rcl_publish(&publisher, &msg, NULL);
        msg.data++;
        gpio_put(25, msg.data % 2); // Toggle LED each publish
        
        sleep_ms(1000);
    }
    
    return 0;
}
