// Copyright (c) Sensrad 2025-2026

#pragma once

#include <vector>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "map_closures/MapClosures.hpp"

namespace vegvisir {

// Forward declaration
class Vegvisir;

// Backend interface (abstract base class for SLAM and Localization modes)
class VegvisirBackend {
 public:
  explicit VegvisirBackend(Vegvisir& vegvisir) : vegvisir_(vegvisir) {}
  virtual ~VegvisirBackend() = default;

  virtual void initialize() = 0;

  // Called at start of update(): sets vegvisir_.current_pose_ and
  // vegvisir_.tf_map_odom_ policy.
  virtual void preIntegrate(const Eigen::Matrix4d& T_odom_base, const Sophus::SE3d& delta_pose) = 0;

  // Called after voxel integration: sets local trajectory policy.
  virtual void postIntegrate() = 0;

  // Query cadence
  virtual double queryDistanceM() const = 0;

  // Called when query cadence triggers: backend builds query clouds and/or
  // performs node/submap actions, then calls
  // vegvisir_.processLoopClosures(...)
  virtual void runQueryCycle(const Eigen::Matrix4d& T_odom_base) = 0;

  // Backend must provide candidate retrieval:
  // SLAM: GetTopKClosures (query + add)
  // Localization: QueryTopKClosures (query-only)
  virtual std::vector<map_closures::ClosureCandidate> retrieveCandidates(
      int query_id, const std::vector<Eigen::Vector3d>& query_points_mc) = 0;

  // Backend must apply accepted closure:
  // SLAM: add factor + optimize keypose graph
  // Localization: Kalman measurement update (uses query_odom_base)
  virtual void applyAcceptedClosure(const map_closures::ClosureCandidate& c,
                                    const Eigen::Matrix4d& query_odom_base) = 0;

 protected:
  Vegvisir& vegvisir_;
};

}  // namespace vegvisir
