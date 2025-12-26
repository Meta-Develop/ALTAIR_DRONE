cmake_minimum_required(VERSION 3.12)

# Include Pico SDK
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_sensors C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_2_sensors
    main.cpp
    ism330.cpp
    mmc5983.cpp
    bmp388.cpp
    vl53l4cx.cpp
)

# Generate PIO header from .pio file
pico_generate_pio_header(pico_2_sensors ${CMAKE_CURRENT_LIST_DIR}/spi_slave.pio)

target_link_libraries(pico_2_sensors
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    hardware_spi
    hardware_i2c
    hardware_gpio
    hardware_timer
)

pico_enable_stdio_usb(pico_2_sensors 1)
pico_enable_stdio_uart(pico_2_sensors 0)

target_include_directories(pico_2_sensors PRIVATE
    ${CMAKE_SOURCE_DIR}/../../common/include
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(pico_2_sensors)
