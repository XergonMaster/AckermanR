cmake_minimum_required(VERSION 3.8)
project(emergency_brake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(eb_node src/eb_node.cpp)
ament_target_dependencies(eb_node rclcpp sensor_msgs nav_msgs geometry_msgs)

install(TARGETS
  eb_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
