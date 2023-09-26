cmake_minimum_required(VERSION 3.8)
project(wall_following_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(wall_following_node src/wall_following_node.cpp)
ament_target_dependencies(wall_following_node rclcpp sensor_msgs std_msgs)

add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp std_msgs geometry_msgs)

# add_executable(follow_gap src/follow_gap.cpp)
# ament_target_dependencies(follow_gap rclcpp geometry_msgs sensor_msgs)

install(TARGETS
  wall_following_node
  control_node
  # follow_gap
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
