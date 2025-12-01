cmake_minimum_required(VERSION 3.8)
project(actuator_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(actuator_bridge_node src/actuator_bridge_node.cpp)
target_include_directories(actuator_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(actuator_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(actuator_bridge_node
  rclcpp
  std_msgs
  sensor_msgs
  dynamixel_sdk
)

install(TARGETS actuator_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
