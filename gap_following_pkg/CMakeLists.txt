cmake_minimum_required(VERSION 3.8)
project(gap_following_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(follow_gap src/follow_the_gap.cpp)
ament_target_dependencies(follow_gap rclcpp geometry_msgs sensor_msgs)

install(TARGETS
  follow_gap
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
