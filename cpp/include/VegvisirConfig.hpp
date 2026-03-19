// Copyright (c) Sensrad 2025-2026

#pragma once

#include <cstddef>

namespace vegvisir {

struct VegvisirConfig {
  double voxel_size = 0.8;
  double splitting_distance_slam = 50.0;
  double splitting_distance_localization = 5.0;
  double overlap_threshold = 0.20;
  int pgo_max_iterations = 10;
  std::size_t inliers_threshold = 25;
};

}  // namespace vegvisir
