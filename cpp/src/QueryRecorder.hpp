// Copyright (c) Sensrad 2026

#pragma once

#include <cstddef>
#include <cstdint>

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "map_closures/MapClosures.hpp"

namespace vegvisir {

// One record per closure candidate evaluated for a single query. Captures the
// candidate's scores and both the pre-ICP (RANSAC) and post-ICP poses so the
// query<->reference alignment can be evaluated offline.
struct QueryMatchRecord {
  int source_id = -1;
  std::size_t number_of_inliers = 0;
  std::size_t sift_inliers = 0;
  std::size_t lbd_inliers = 0;
  double weighted_score = 0.0;
  Eigen::Matrix4d ransac_pose = Eigen::Matrix4d::Identity();   // pose before ICP
  Eigen::Matrix4d refined_pose = Eigen::Matrix4d::Identity();  // pose after ICP
  bool icp_converged = false;
  bool accepted = false;  // passed overlap validation
};

// Writes per-localization-query data to disk for offline alignment evaluation.
// Each query produces a self-contained subdirectory holding the query clouds,
// the query ground plane, the query density map, and a metadata file.
//
// Thread-safety: all calls happen on the single background closure thread, so
// no internal synchronization is required.
class LocalizationQueryRecorder {
 public:
  explicit LocalizationQueryRecorder(std::string output_dir);

  // Dump one query. query_points_mc is the local map fed to map-closure
  // detection; query_points_icp is the per-voxel cloud used for ICP. artifacts
  // carries the query ground plane and density map (may be invalid if the query
  // produced no density map).
  void record(uint64_t timestamp_ns, int query_id, const Eigen::Matrix4d& query_odom_base,
              const Eigen::Matrix4d& tf_map_odom, double voxel_size,
              const std::vector<Eigen::Vector3d>& query_points_mc,
              const std::vector<Eigen::Vector3d>& query_points_icp,
              const map_closures::QueryArtifacts& artifacts,
              const std::vector<QueryMatchRecord>& matches);

 private:
  std::string output_dir_;
  uint64_t query_counter_ = 0;
};

}  // namespace vegvisir
