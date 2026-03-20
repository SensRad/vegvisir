# Copyright (c) Sensrad 2025-2026

add_library(pgo STATIC pose_graph_optimizer.cpp)
target_include_directories(pgo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..
                                      ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pgo PUBLIC g2o::core g2o::types_slam3d
                                 g2o::solver_cholmod Eigen3::Eigen)
target_compile_features(pgo PUBLIC cxx_std_20)
