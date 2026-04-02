// Copyright (c) Sensrad 2026

#pragma once

#include <vector>

#include <Eigen/Dense>

#include "VegvisirBackend.hpp"

namespace vegvisir {

// Forward declaration
class Vegvisir;

class LocalizationBackend final : public VegvisirBackend {
 public:
  explicit LocalizationBackend(Vegvisir& vegvisir);

  void initialize() override;

  void updatePoseEstimate(const Eigen::Matrix4d& pose_odom_base,
                          const Sophus::SE3d& delta_pose) override;

  void updateTrajectory() override;

  [[nodiscard]] double queryDistanceM() const override;

  void runQueryCycle(const Eigen::Matrix4d& pose_odom_base) override;

  std::vector<map_closures::ClosureCandidate> retrieveCandidates(
      int query_id, const std::vector<Eigen::Vector3d>& query_points_mc) override;

  void applyAcceptedClosure(const map_closures::ClosureCandidate& c,
                            const Eigen::Matrix4d& query_odom_base) override;

 private:
  void handleClosureMeasurementUpdate(int source_id, const Eigen::Matrix4d& pose,
                                      const Eigen::Matrix4d& query_odom_base);
  void initLocalizationAnchor(const Eigen::Matrix4d& pose_odom_base);
  void pruneLocalizationSubmapBuffer();
  void cutLocalizationSubmap();
  void buildLocalizationQueryCloudInBaseFrame(const Eigen::Matrix4d& pose_odom_base,
                                              std::vector<Eigen::Vector3d>& query_points_mc,
                                              std::vector<Eigen::Vector3d>& query_points_icp);

  // Localization-only state
  Eigen::Matrix4d pose_odom_anchor_ = Eigen::Matrix4d::Identity();  // odom <- anchor
  bool localization_anchor_initialized_ = false;

  static constexpr double DETERMINANT_TOLERANCE = 1e-2;
};

}  // namespace vegvisir
