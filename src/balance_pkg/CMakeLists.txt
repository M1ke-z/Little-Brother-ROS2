cmake_minimum_required(VERSION 3.8)
project(balance_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)

add_executable(balance_node 
  src/balance_node.cpp
)

# target_include_directories(balance_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

ament_target_dependencies(balance_node
  rclcpp
  sensor_msgs
  interfaces
)

# ament_export_dependencies(rosidl_default_runtime)

install( TARGETS
  balance_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
