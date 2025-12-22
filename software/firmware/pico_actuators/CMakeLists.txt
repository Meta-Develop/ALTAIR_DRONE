cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_FETCH_FROM_GIT ON)
include(pico_sdk_import.cmake)

project(pico_actuators C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Configure for Pico 2
set(PICO_BOARD pico2)

# Initialize Pico SDK
set(PICO_NO_PICOTOOL 1)
pico_sdk_init()


# =========================================
# Pico Actuators Firmware
# =========================================
add_compile_definitions(NUM_ESC=6)

add_executable(pico_actuators

    src/main.c
    src/drivers/dshot.c
    src/drivers/telemetry.c
)

target_include_directories(pico_actuators PRIVATE src src/drivers)

target_link_libraries(pico_actuators 
    pico_stdlib
    hardware_spi
    hardware_dma
    hardware_gpio
    hardware_pio
    hardware_timer
    hardware_pwm
)

# No USB stdio needed for SPI-only firmware
pico_enable_stdio_usb(pico_actuators 0)
pico_enable_stdio_uart(pico_actuators 0)

# PIO Generation
pico_generate_pio_header(pico_actuators ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/dshot.pio)
pico_generate_pio_header(pico_actuators ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/telemetry.pio)

pico_add_extra_outputs(pico_actuators)
