cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(altair_pico_firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize Pico SDK
pico_sdk_init()

# Fetch micro-ROS for Raspberry Pi Pico
include(FetchContent)
FetchContent_Declare(
  pico_micro_ros
  GIT_REPOSITORY https://github.com/micro-ROS/micro_ros_raspberrypi
  GIT_TAG main
)

# This will download the library and its dependencies (including FreeRTOS)
FetchContent_MakeAvailable(pico_micro_ros)

# Add the executable
add_executable(pico_node
    src/main.c
    src/pico_node.c
    src/pico_uart_transports.c
    src/drivers/dshot.c
    src/drivers/bno055.c
)

# Include directories
target_include_directories(pico_node PRIVATE 
    src 
    src/drivers
)

# Generate PIO header
pico_generate_pio_header(pico_node ${CMAKE_CURRENT_LIST_DIR}/src/drivers/dshot.pio)

# Link libraries
target_link_libraries(pico_node 
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_i2c
    hardware_dma
    pico_micro_ros
)

# Enable USB output (for micro-ROS transport)
pico_enable_stdio_usb(pico_node 1)
pico_enable_stdio_uart(pico_node 0)

# Create UF2 output
pico_add_extra_outputs(pico_node)
