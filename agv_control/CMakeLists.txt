cmake_minimum_required(VERSION 3.5)
project(agv_control)

# Add C++14 standard
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED) # Agrega Eigen3 como dependencia

include_directories(${PROJECT_SOURCE_DIR}/lib)
include_directories(${EIGEN3_INCLUDE_DIR}) # Incluye los directorios de Eigen

add_library(serialib STATIC lib/serialib.cpp)

# agv_controller
add_executable(agv_controller src/agv_controller.cpp)

target_sources(agv_controller PRIVATE
  src/agv_controller.cpp
)

ament_target_dependencies(agv_controller
  rclcpp
  geometry_msgs
)

# Enlaza la biblioteca serialib al ejecutable agv_controller
target_link_libraries(agv_controller
  serialib
)

# odom_node
add_executable(odom_node src/odom_node.cpp) # Añade un ejecutable para odom_node

target_sources(odom_node PRIVATE
  src/odom_node.cpp
)

# Agrega las dependencias necesarias para odom_node
ament_target_dependencies(odom_node
  rclcpp
  std_msgs
  nav_msgs
  Eigen3 # Añade Eigen3 como dependencia de enlace
)

install(TARGETS 
  agv_controller
  odom_node # Instala odom_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
