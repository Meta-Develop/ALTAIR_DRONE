cmake_minimum_required(VERSION 3.8)
project(pico_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImuBatch.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

add_executable(pico_bridge_node src/pico_bridge_node.cpp)

target_include_directories(pico_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/include
)

find_library(GPIOD_LIB gpiod REQUIRED)
target_link_libraries(pico_bridge_node ${GPIOD_LIB})

target_compile_features(pico_bridge_node PUBLIC cxx_std_17)

# Use ament_target_dependencies for proper ROS 2 includes
rosidl_target_interfaces(pico_bridge_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(pico_bridge_node rclcpp sensor_msgs)

install(TARGETS pico_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
