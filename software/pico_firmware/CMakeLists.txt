cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_FETCH_FROM_GIT ON)
include(pico_sdk_import.cmake)

project(altair_pico_firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize Pico SDK
pico_sdk_init()

# Set PICO_SDK_PATH environment variable for the child project
set(ENV{PICO_SDK_PATH} "${PICO_SDK_PATH}")

include(FetchContent)

# Fetch micro-ROS for Raspberry Pi Pico
FetchContent_Declare(
  pico_micro_ros
  GIT_REPOSITORY https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk
  GIT_TAG 6.0.0
)

# Fetch FreeRTOS Kernel
FetchContent_Declare(
  FreeRTOS-Kernel
  GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
  GIT_TAG V11.0.1
)

# Define FreeRTOS Config target (must be before adding FreeRTOS-Kernel)
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE src)
target_compile_definitions(freertos_config INTERFACE configNUM_CORES=2)
target_link_libraries(freertos_config INTERFACE pico_stdlib)

# Set FreeRTOS Port and Heap
set(FREERTOS_PORT GCC_RP2040 CACHE STRING "" FORCE)
set(FREERTOS_HEAP 4 CACHE STRING "" FORCE)

FetchContent_MakeAvailable(pico_micro_ros FreeRTOS-Kernel)

# Fix FreeRTOS Port linking
if(TARGET freertos_kernel_port)
    target_link_libraries(freertos_kernel_port PUBLIC pico_stdlib)
endif()

# Add the executable
add_executable(pico_node
    src/main.c
    src/pico_node.c
    src/pico_uart_transports.c
    src/drivers/dshot.c
    src/drivers/bno055.c
    src/drivers/telemetry.c
)

# Add event_groups.c from FreeRTOS to fix linking issue
# The FreeRTOS port references event groups but the build system doesn't include it by default
target_sources(pico_node PRIVATE
    ${CMAKE_BINARY_DIR}/_deps/freertos-kernel-src/event_groups.c
)

# Include directories
target_include_directories(pico_node PRIVATE 
    src 
    src/drivers
    ${pico_micro_ros_SOURCE_DIR}/libmicroros/include
)

# Link directories for micro-ROS
target_link_directories(pico_node PRIVATE ${pico_micro_ros_SOURCE_DIR}/libmicroros)

# Generate PIO header
pico_generate_pio_header(pico_node ${CMAKE_CURRENT_LIST_DIR}/src/drivers/dshot.pio)
pico_generate_pio_header(pico_node ${CMAKE_CURRENT_LIST_DIR}/src/drivers/telemetry.pio)

# Link libraries
target_link_libraries(pico_node 
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_i2c
    hardware_dma
    microros
    freertos_kernel
)

# Enable USB output (for micro-ROS transport)
pico_enable_stdio_usb(pico_node 1)
pico_enable_stdio_uart(pico_node 0)

# Create UF2 output
pico_add_extra_outputs(pico_node)
