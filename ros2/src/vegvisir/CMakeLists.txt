# Copyright (c) Sensrad 2025-2026

cmake_minimum_required(VERSION 3.16...3.26)
project(vegvisir)

# Default VEGVISIR_BUILD_NODE to ON when top-level (colcon), OFF when consumed
# via add_subdirectory()
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  option(VEGVISIR_BUILD_NODE "Build the ROS2 vegvisir_node executable" ON)
else()
  option(VEGVISIR_BUILD_NODE "Build the ROS2 vegvisir_node executable" OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# System dependencies
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)

# Fetched dependencies
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FetchG2O.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FetchSophus.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FetchTessil.cmake)

# pgo (pose graph optimizer, C++20)
add_library(pgo STATIC pgo/pose_graph_optimizer.cpp)
target_include_directories(pgo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/pgo
                                      ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pgo PUBLIC g2o::core g2o::types_slam3d
                                 g2o::solver_cholmod Eigen3::Eigen)
target_compile_features(pgo PUBLIC cxx_std_20)

# voxel_map
add_library(voxel_map STATIC voxel_map/VoxelMap.cpp voxel_map/VoxelUtils.cpp)
target_include_directories(voxel_map PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(voxel_map PUBLIC Eigen3::Eigen tsl::robin_map)

# map_closures
add_subdirectory(map_closures)

# vegvisir (core library)
add_library(
  vegvisir STATIC
  src/Vegvisir.cpp
  src/VegvisirIO.cpp
  src/LocalMapGraph.cpp
  src/PoseKalmanFilter.cpp
  src/SlamBackend.cpp
  src/LocalizationBackend.cpp
  icp_svd/IcpSvd.cpp)
target_include_directories(vegvisir PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                           ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
  vegvisir
  PUBLIC voxel_map
         map_closures
         pgo
         Sophus::Sophus
         TBB::tbb
         ${OpenCV_LIBS}
         Eigen3::Eigen)

# vegvisir_node (ROS2 executable, opt-in)
if(VEGVISIR_BUILD_NODE)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(oden_interfaces REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)

  add_executable(vegvisir_node src/main.cpp src/VegvisirNode.cpp)
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
endif()
