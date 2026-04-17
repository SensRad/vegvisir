// Copyright (c) Sensrad 2026

#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "VegvisirBackend.hpp"
#include "pgo/pose_graph_optimizer.hpp"

namespace vegvisir {

// Forward declaration
class Vegvisir;

class SlamBackend final : public VegvisirBackend {
 public:
  explicit SlamBackend(Vegvisir& vegvisir);

  void initialize() override;

  void updatePoseEstimate(const Eigen::Matrix4d& pose_odom_base, const Sophus::SE3d& delta_pose,
                          uint64_t timestamp_ns) override;

  void updateTrajectory() override;

  [[nodiscard]] double queryDistanceM() const override;

  void runQueryCycle(const Eigen::Matrix4d& pose_odom_base, uint64_t timestamp_ns) override;

  std::vector<map_closures::ClosureCandidate> retrieveCandidates(
      int query_id, const std::vector<Eigen::Vector3d>& query_points_mc) override;

  void applyAcceptedClosure(const map_closures::ClosureCandidate& c,
                            const Eigen::Matrix4d& query_odom_base) override;

 private:
  void optimizeKeyposeGraph();
  void generateNewNode(const Eigen::Matrix4d& pose_odom_base, uint64_t timestamp_ns);

  // SLAM-only state
  std::vector<Eigen::Matrix4d> pose_at_nodes_;
  std::unique_ptr<pgo::PoseGraphOptimizer> keypose_optimizer_;
};

}  // namespace vegvisir
