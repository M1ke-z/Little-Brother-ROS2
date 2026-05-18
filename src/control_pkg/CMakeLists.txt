cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)


add_executable(control_node 
  src/control_node.cpp
  src/control_parameters.cpp
)

target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(control_node
  rclcpp
  std_msgs
  sensor_msgs
  interfaces
  trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install( TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)



ament_package()