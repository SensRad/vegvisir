# Copyright (c) Sensrad 2025-2026

add_library(voxel_map STATIC VoxelMap.cpp VoxelUtils.cpp)
target_include_directories(voxel_map PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(voxel_map PUBLIC Eigen3::Eigen tsl::robin_map)
