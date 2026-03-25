// Copyright (c) Sensrad 2026

#pragma once

#include <vector>

#include <Eigen/Dense>

namespace vegvisir {

struct GnssMeasurement {
  int pose_index;
  Eigen::Vector3d position_enu;
  Eigen::Matrix3d information_matrix;
};

struct GnssPoseMeasurement {
  int pose_index;
  Eigen::Matrix4d pose_enu;
  Eigen::Matrix<double, 6, 6> information_matrix;
};

class GnssState {
 public:
  void addMeasurement(int pose_index, const Eigen::Vector3d& position_enu,
                      const Eigen::Matrix3d& information_matrix) {
    measurements_.push_back({pose_index, position_enu, information_matrix});
  }

  void clearMeasurements() { measurements_.clear(); }

  [[nodiscard]] const std::vector<GnssMeasurement>& measurements() const { return measurements_; }

  [[nodiscard]] size_t numMeasurements() const { return measurements_.size(); }

  void addPoseMeasurement(int pose_index, const Eigen::Matrix4d& pose_enu,
                          const Eigen::Matrix<double, 6, 6>& information_matrix) {
    pose_measurements_.push_back({pose_index, pose_enu, information_matrix});
  }

  void clearPoseMeasurements() { pose_measurements_.clear(); }

  [[nodiscard]] const std::vector<GnssPoseMeasurement>& poseMeasurements() const {
    return pose_measurements_;
  }

  [[nodiscard]] size_t numPoseMeasurements() const { return pose_measurements_.size(); }

  [[nodiscard]] bool hasGnss() const {
    return !measurements_.empty() || !pose_measurements_.empty();
  }

  // Alignment state
  Eigen::Matrix4d optimized_pose_enu_map = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d initial_pose_enu_map = Eigen::Matrix4d::Identity();
  bool has_initial_alignment = false;

 private:
  std::vector<GnssMeasurement> measurements_;
  std::vector<GnssPoseMeasurement> pose_measurements_;
};

}  // namespace vegvisir
