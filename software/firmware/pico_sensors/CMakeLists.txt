cmake_minimum_required(VERSION 3.13)

# Initialize SDK
include(pico_sdk_import.cmake)

project(pico_sensors C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# 1. Safe Mode Firmware (Polling + Sync Bypass)
add_executable(pico_sensors src/main.c)
target_include_directories(pico_sensors PRIVATE src src/drivers)
target_link_libraries(pico_sensors 
    pico_stdlib 
    hardware_spi 
    hardware_gpio 
    hardware_dma
    hardware_pio
)
pico_enable_stdio_usb(pico_sensors 1)
pico_add_extra_outputs(pico_sensors)

# 2. High-Performance DMA Firmware (with 9DoF Sensors)
add_executable(pico_sensors_dma 
    src/main_dma.c
    src/drivers/ism330dhcx.c
    src/drivers/mmc5983ma.c
)
target_include_directories(pico_sensors_dma PRIVATE src src/drivers)
target_link_libraries(pico_sensors_dma 
    pico_stdlib 
    hardware_spi 
    hardware_gpio 
    hardware_dma
    hardware_pio
    hardware_timer
)
pico_generate_pio_header(pico_sensors_dma ${CMAKE_CURRENT_LIST_DIR}/src/spi_slave.pio)
target_include_directories(pico_sensors_dma PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

pico_enable_stdio_usb(pico_sensors_dma 1)
pico_add_extra_outputs(pico_sensors_dma)

# 3. Blink Test (Flash Verification)
add_executable(blink_test src/blink_test.c)
target_link_libraries(blink_test pico_stdlib hardware_gpio)
pico_add_extra_outputs(blink_test)

# 4. Pin Sweep Diagnostic
add_executable(pin_sweep src/pin_sweep.c)
target_link_libraries(pin_sweep pico_stdlib)
pico_enable_stdio_usb(pin_sweep 1)
pico_add_extra_outputs(pin_sweep)
