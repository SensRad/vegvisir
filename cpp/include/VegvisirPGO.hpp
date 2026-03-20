// Copyright (c) Sensrad 2025-2026

#pragma once

#include <vector>

#include <Eigen/Dense>

#include "GnssState.hpp"
#include "LocalMapGraph.hpp"
#include "VegvisirConfig.hpp"
#include "map_closures/MapClosures.hpp"

namespace vegvisir {

struct FineGrainedPGOResult {
  std::vector<Eigen::Matrix4d> optimized_poses;
  Eigen::Matrix4d alignment_transform = Eigen::Matrix4d::Identity();
};

FineGrainedPGOResult runFineGrainedPGO(const LocalMapGraph& local_map_graph,
                                       const std::vector<map_closures::ClosureCandidate>& closures,
                                       GnssState& gnss_state, const VegvisirConfig& config);

}  // namespace vegvisir
