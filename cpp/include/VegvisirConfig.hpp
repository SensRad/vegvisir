// Copyright (c) Sensrad 2026

#pragma once

#include <cstddef>

#include <string>

namespace vegvisir {

struct VegvisirConfig {
  double voxel_size = 0.8;
  double splitting_distance_slam = 50.0;
  double splitting_distance_localization = 5.0;
  double overlap_threshold = 0.20;
  int pgo_max_iterations = 10;
  std::size_t inliers_threshold = 25;

  // ICP refinement parameters
  double icp_refinement_voxel_size = 0.1;
  int icp_max_iterations = 500;
  double icp_convergence_criterion = 5e-5;
  double icp_max_correspondence_distance = 0.8;

  // Debug/evaluation: optionally dump each localization query to disk. Off by
  // default so the production path is unaffected.
  bool save_localization_queries = false;
  std::string query_dump_dir;
};

}  // namespace vegvisir
