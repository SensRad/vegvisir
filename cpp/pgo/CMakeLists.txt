# Copyright (c) Sensrad 2026

add_library(pgo STATIC pose_graph_optimizer.cpp)
target_include_directories(pgo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(
  pgo
  PUBLIC g2o::core Eigen3::Eigen
  PRIVATE g2o::types_slam3d g2o::solver_cholmod)
