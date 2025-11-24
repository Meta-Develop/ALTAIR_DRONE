cmake_minimum_required(VERSION 3.8)
project(altair_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# Controller Node
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node Eigen3::Eigen)
ament_target_dependencies(controller_node
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  urdf
  Eigen3
)

install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
