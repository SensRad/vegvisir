# Copyright (c) Sensrad 2026 Copyright (c) 2024 Saurabh Gupta, Tiziano
# Guadagnino, Benedikt Mersch, Ignacio Vizzo, Cyrill Stachniss.
# SPDX-License-Identifier: MIT

add_library(map_closures STATIC)
target_sources(
  map_closures PRIVATE DensityMap.cpp AlignRansac2D.cpp GroundAlign.cpp
                       SiftFeatureLayer.cpp LbdFeatureLayer.cpp MapClosures.cpp)
target_include_directories(map_closures PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(
  map_closures
  PUBLIC Eigen3::Eigen ${OpenCV_LIBS}
  PRIVATE Sophus::Sophus)
if(COMMAND set_global_target_properties)
  set_global_target_properties(map_closures)
endif()
