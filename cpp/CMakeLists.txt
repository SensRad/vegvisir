# Copyright (c) Sensrad 2025-2026

cmake_minimum_required(VERSION 3.16...3.26)

# Only declare project() when built standalone (not via add_subdirectory)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(vegvisir_cpp LANGUAGES CXX)
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
