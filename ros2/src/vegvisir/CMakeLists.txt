# Copyright (c) Sensrad 2025-2026

cmake_minimum_required(VERSION 3.16...3.26)
project(vegvisir)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(oden_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ core libraries (from cpp/ directory)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp
                 ${CMAKE_CURRENT_BINARY_DIR}/vegvisir_cpp)

# vegvisir_node (ROS2 executable)
add_executable(vegvisir_node src/main.cpp src/VegvisirNode.cpp)
target_include_directories(vegvisir_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(vegvisir_node vegvisir)
ament_target_dependencies(
  vegvisir_node
  rclcpp
  sensor_msgs
  oden_interfaces
  message_filters
  tf2_ros
  geometry_msgs
  nav_msgs
  visualization_msgs)

install(TARGETS vegvisir_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
