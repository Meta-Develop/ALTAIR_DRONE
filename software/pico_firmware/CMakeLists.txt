cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_FETCH_FROM_GIT ON)
include(pico_sdk_import.cmake)

project(altair_pico_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize Pico SDK
pico_sdk_init()

# Fetch Micro-ROS Pico SDK
include(FetchContent)
FetchContent_Declare(
    micro_ros_raspberrypi_pico_sdk
    GIT_REPOSITORY https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk
    GIT_TAG humble
)
FetchContent_MakeAvailable(micro_ros_raspberrypi_pico_sdk)

# Include Micro-ROS directory
include_directories(${micro_ros_raspberrypi_pico_sdk_SOURCE_DIR}/lib/include)
include_directories(${micro_ros_raspberrypi_pico_sdk_SOURCE_DIR}/src)

# Add Executable
add_executable(pico_node
    src/main.c
    src/pico_node.c
    src/drivers/dshot.c
    src/drivers/telemetry.c
    src/drivers/mpu6050.c
)

# PIO Generation
pico_generate_pio_header(pico_node ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/dshot.pio)
pico_generate_pio_header(pico_node ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/telemetry.pio)

# Include Dirs
target_include_directories(pico_node PRIVATE 
    src
    src/drivers
    ${CMAKE_CURRENT_BINARY_DIR}/src/drivers # For generated PIO headers
)

# Link Libraries
target_link_libraries(pico_node 
    pico_stdlib
    hardware_i2c
    hardware_pio
    hardware_dma
    hardware_gpio
    
    # Micro-ROS
    pico_microros
)

# Enable USB CDC for Micro-ROS (Disable UART)
pico_enable_stdio_usb(pico_node 1)
pico_enable_stdio_uart(pico_node 0)

# Build Ops
pico_add_extra_outputs(pico_node)
