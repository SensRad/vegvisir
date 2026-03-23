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
find_package(OpenCV REQUIRED)

# Fetched dependencies
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FetchG2O.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FetchSophus.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FetchTessil.cmake)

# Sub-libraries
add_subdirectory(pgo)
add_subdirectory(voxel_map)
add_subdirectory(icp_svd)
add_subdirectory(map_closures)
add_subdirectory(kalman_filter)

# vegvisir (core library)
add_library(
  vegvisir STATIC
  src/Vegvisir.cpp
  src/VegvisirIO.cpp
  src/VegvisirPGO.cpp
  src/LocalMapGraph.cpp
  src/VegvisirBackend.cpp
  src/SlamBackend.cpp
  src/LocalizationBackend.cpp)
target_include_directories(vegvisir PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                           ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
  vegvisir
  PUBLIC voxel_map
         icp_svd
         map_closures
         pgo
         kalman_filter
         Sophus::Sophus
         Eigen3::Eigen)
