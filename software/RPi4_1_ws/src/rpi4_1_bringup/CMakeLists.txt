cmake_minimum_required(VERSION 3.8)
project(rpi4_1_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(altair_interfaces REQUIRED)

include_directories(../../../common) 

add_executable(spi_bridge_cpp src/spi_bridge.cpp)
ament_target_dependencies(spi_bridge_cpp rclcpp std_msgs sensor_msgs altair_interfaces)
target_link_libraries(spi_bridge_cpp gpiod)

install(TARGETS
  spi_bridge_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/spi_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spi_bridge_py
)

ament_package()
