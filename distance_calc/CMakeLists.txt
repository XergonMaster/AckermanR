cmake_minimum_required(VERSION 3.8)
project(distance_calc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)  # Añadido rcl_interfaces aquí

add_executable(emergency_break src/emergency_break.cpp)

# Añade las bibliotecas de rcl_interfaces a las rutas de inclusión
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
)

target_compile_features(emergency_break PUBLIC c_std_99 cxx_std_17)
target_link_libraries(emergency_break rclcpp sensor_msgs nav_msgs rcl_interfaces)

install(TARGETS emergency_break
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()