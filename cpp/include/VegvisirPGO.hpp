// Copyright (c) Sensrad 2026

#pragma once

#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "map_closures/MapClosures.hpp"

namespace vegvisir {

class GnssState;
class LocalMapGraph;
struct VegvisirConfig;

struct FineGrainedPGOResult {
  std::vector<Eigen::Matrix4d> optimized_poses;
  std::unordered_map<int, Eigen::Matrix4d> optimized_keyposes;
  Eigen::Matrix4d alignment_transform = Eigen::Matrix4d::Identity();
};

FineGrainedPGOResult runFineGrainedPGO(const LocalMapGraph& local_map_graph,
                                       const std::vector<map_closures::ClosureCandidate>& closures,
                                       const GnssState& gnss_state, const VegvisirConfig& config);

}  // namespace vegvisir
