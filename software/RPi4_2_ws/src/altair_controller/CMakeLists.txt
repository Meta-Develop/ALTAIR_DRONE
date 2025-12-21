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

# Acados Config
# Assuming ACADOS_SOURCE_DIR env var or standard path
set(ACADOS_PATH "/opt/acados" CACHE PATH "Path to Acados")
include_directories(${ACADOS_PATH}/include)
include_directories(${ACADOS_PATH}/include/blasfeo/include)
include_directories(${ACADOS_PATH}/include/hpipm/include)

link_directories(${ACADOS_PATH}/lib)

# Generated Code Path (Standard Output: c_generated_code)
# We assume the user generates code into 'src/c_generated_code'
file(GLOB ACADOS_GENERATED_SRCS 
    "src/c_generated_code/altair_drone_model/altair_drone_expl_vde_forw.c"
    "src/c_generated_code/altair_drone_model/altair_drone_expl_ode_fun.c"
    "src/c_generated_code/acados_solver_altair_drone.c"
)
include_directories(src/c_generated_code)

# Controller Node
add_executable(controller_node src/controller_node.cpp ${ACADOS_GENERATED_SRCS})
target_link_libraries(controller_node 
  Eigen3::Eigen
  acados
  hpipm
  blasfeo
)
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
