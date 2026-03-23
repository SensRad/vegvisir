# Copyright (c) Sensrad 2025-2026

add_library(icp_svd STATIC IcpSvd.cpp)
target_include_directories(icp_svd PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(icp_svd PUBLIC voxel_map Eigen3::Eigen)
