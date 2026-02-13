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

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/MovementData.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}  
)

add_executable(control_node src/control_node.cpp)

ament_target_dependencies(
  control_node
  rclcpp
  std_msgs
  sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()