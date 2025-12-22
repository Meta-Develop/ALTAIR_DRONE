cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_FETCH_FROM_GIT ON)
include(pico_sdk_import.cmake)

project(altair_pico_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


# Initialize Pico SDK
set(PICO_NO_PICOTOOL 1)
pico_sdk_init()

option(BUILD_MICROROS "Build Micro-ROS Firmware" OFF)

if(BUILD_MICROROS)
    # Ensure Enviroment Variable is set for Micro-ROS SDK
    set(ENV{PICO_SDK_PATH} ${PICO_SDK_PATH})

    # Fetch Micro-ROS Pico SDK
    include(FetchContent)
    FetchContent_Declare(
        micro_ros_raspberrypi_pico_sdk
        GIT_REPOSITORY https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk
        GIT_TAG humble
    )
    FetchContent_MakeAvailable(micro_ros_raspberrypi_pico_sdk)

    # Fetch FreeRTOS (Required for headers)
    set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE PATH "Path to FreeRTOSConfig.h" FORCE)
    set(FREERTOS_PORT "GCC_RP2040" CACHE STRING "FreeRTOS Port" FORCE)
    FetchContent_Declare(
        FreeRTOS-Kernel
        GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
        GIT_TAG V11.1.0
    )
    FetchContent_MakeAvailable(FreeRTOS-Kernel)
    # Link freertos_kernel to pico_stdlib so it can find pico/stdlib.h
    target_link_libraries(freertos_kernel PUBLIC pico_stdlib)
    target_link_libraries(freertos_kernel_port PUBLIC pico_stdlib)

    # Include Micro-ROS directory
    include_directories(${micro_ros_raspberrypi_pico_sdk_SOURCE_DIR}/libmicroros/include)
    include_directories(${micro_ros_raspberrypi_pico_sdk_SOURCE_DIR})
    link_directories(${micro_ros_raspberrypi_pico_sdk_SOURCE_DIR}/libmicroros)
    
    # Include FreeRTOS directories (Note: FetchContent lowercases the name)
    include_directories(${freertos-kernel_SOURCE_DIR}/include)
    include_directories(${freertos-kernel_SOURCE_DIR}/portable/GCC/ARM_CM0)
    # Fallback if variable is empty
    include_directories(${CMAKE_BINARY_DIR}/_deps/freertos-kernel-src/include)
    include_directories(${CMAKE_BINARY_DIR}/_deps/freertos-kernel-src/portable/GCC/ARM_CM0)

    # =========================================
    # PRODUCTION FIRMWARE (Superloop, No FreeRTOS)
    # =========================================
    add_executable(pico_node
        src/pico_node_full.c
        src/drivers/mpu6050.c
        src/drivers/dshot.c
        src/drivers/telemetry.c
        ${micro_ros_raspberrypi_pico_sdk_SOURCE_DIR}/pico_uart_transport.c
    )

    # PIO Generation
    pico_generate_pio_header(pico_node ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/dshot.pio)
    pico_generate_pio_header(pico_node ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/telemetry.pio)

    target_include_directories(pico_node PRIVATE 
        src
        src/drivers
        ${CMAKE_CURRENT_BINARY_DIR} # For generated PIO headers
        ${micro_ros_raspberrypi_pico_sdk_SOURCE_DIR}/libmicroros/include
        ${micro_ros_raspberrypi_pico_sdk_SOURCE_DIR}
    )

    target_link_libraries(pico_node 
        pico_stdlib
        hardware_i2c
        hardware_timer
        hardware_pio
        hardware_dma
        hardware_gpio
        microros
    )

    # Enable USB CDC for Micro-ROS (Disable UART)
    pico_enable_stdio_usb(pico_node 1)
    pico_enable_stdio_uart(pico_node 0)
    
    # Micro-ROS specific definitions
    # NOTE: Increased heap to prevent crash during publisher init (DDS buffers)
    target_compile_definitions(pico_node PRIVATE
        PICO_UART_ENABLE_CRLF_SUPPORT=0
        PICO_STDIO_ENABLE_CRLF_SUPPORT=0
        PICO_STDIO_DEFAULT_CRLF=0
        PICO_HEAP_SIZE=0x10000
    )

    # Build Ops
    pico_add_extra_outputs(pico_node)
    
    # =========================================
    # Simple Test Firmware (No FreeRTOS)
    # =========================================
    add_executable(pico_node_simple
        src/pico_node_simple.c
        ${micro_ros_raspberrypi_pico_sdk_SOURCE_DIR}/pico_uart_transport.c
    )
    
    target_include_directories(pico_node_simple PRIVATE
        src
        ${micro_ros_raspberrypi_pico_sdk_SOURCE_DIR}/libmicroros/include
        ${micro_ros_raspberrypi_pico_sdk_SOURCE_DIR}
    )
    
    target_link_libraries(pico_node_simple
        pico_stdlib
        microros
    )
    
    pico_enable_stdio_usb(pico_node_simple 1)
    pico_enable_stdio_uart(pico_node_simple 0)
    
    target_compile_definitions(pico_node_simple PRIVATE
        PICO_UART_ENABLE_CRLF_SUPPORT=0
        PICO_STDIO_ENABLE_CRLF_SUPPORT=0
        PICO_STDIO_DEFAULT_CRLF=0
        PICO_HEAP_SIZE=0x10000
    )
    
    pico_add_extra_outputs(pico_node_simple)
    
    # =========================================
    # Full Firmware (Superloop, No FreeRTOS)
    # =========================================

endif()

# Raw Diagnostic Firmware
add_executable(pico_raw
    src/raw_main.c
    src/drivers/mpu6050.c
)
# Includes
target_include_directories(pico_raw PRIVATE src src/drivers)
# Links
target_link_libraries(pico_raw pico_stdlib hardware_i2c)
# USB
pico_enable_stdio_usb(pico_raw 1)
pico_enable_stdio_uart(pico_raw 0)

pico_add_extra_outputs(pico_raw)

# =========================================
# SPI Slave Firmware (No Micro-ROS, No FreeRTOS)
# =========================================
add_executable(pico_spi
    src/pico_node_spi.c
    src/drivers/mpu6050.c
)
target_include_directories(pico_spi PRIVATE src src/drivers)
target_link_libraries(pico_spi 
    pico_stdlib
    hardware_i2c
    hardware_spi
    hardware_dma
    hardware_gpio
)
# No USB stdio needed for SPI-only firmware
pico_enable_stdio_usb(pico_spi 0)
pico_enable_stdio_uart(pico_spi 0)

pico_add_extra_outputs(pico_spi)
