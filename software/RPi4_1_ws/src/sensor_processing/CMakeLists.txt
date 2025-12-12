cmake_minimum_required(VERSION 3.8)
project(sensor_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(notch_filter_node src/notch_filter_node.cpp)
target_include_directories(notch_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(notch_filter_node PUBLIC cxx_std_17)
ament_target_dependencies(notch_filter_node
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(sensor_processing_node src/sensor_processing_node.cpp)
target_include_directories(sensor_processing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sensor_processing_node PUBLIC cxx_std_17)
ament_target_dependencies(sensor_processing_node
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(imu_unpacker_node src/imu_unpacker_node.cpp)
target_include_directories(imu_unpacker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_unpacker_node PUBLIC cxx_std_17)
ament_target_dependencies(imu_unpacker_node
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(mpu6050_ekf_node src/mpu6050_ekf_node.cpp)
target_include_directories(mpu6050_ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(mpu6050_ekf_node PUBLIC cxx_std_17)
ament_target_dependencies(mpu6050_ekf_node
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
)

install(TARGETS 
  notch_filter_node
  sensor_processing_node
  imu_unpacker_node
  mpu6050_ekf_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
