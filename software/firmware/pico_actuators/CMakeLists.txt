cmake_minimum_required(VERSION 3.13)

# Initialize SDK
include(pico_sdk_import.cmake)

project(pico_actuators C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Actuator Firmware Target
add_executable(pico_actuators 
    src/main.c
    src/drivers/dshot.c
    # src/drivers/telemetry.c # To be created
)

target_include_directories(pico_actuators PRIVATE src src/drivers ../../common)

target_link_libraries(pico_actuators 
    pico_stdlib 
    hardware_spi 
    hardware_dma
    hardware_pio
    hardware_uart
    hardware_irq
    hardware_gpio
    hardware_timer
)

# Generate PIO Header for SPI Slave
pico_generate_pio_header(pico_actuators ${CMAKE_CURRENT_LIST_DIR}/src/spi_slave.pio)

# Generate PIO Header for DShot
pico_generate_pio_header(pico_actuators ${CMAKE_CURRENT_LIST_DIR}/src/drivers/dshot.pio)
pico_generate_pio_header(pico_actuators ${CMAKE_CURRENT_LIST_DIR}/src/drivers/dshot_bidir.pio)
# pico_generate_pio_header(pico_actuators ${CMAKE_CURRENT_LIST_DIR}/src/drivers/uart_rx.pio)


target_include_directories(pico_actuators PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

pico_enable_stdio_usb(pico_actuators 1)
pico_add_extra_outputs(pico_actuators)
