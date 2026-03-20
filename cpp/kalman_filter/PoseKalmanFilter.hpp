// Copyright (c) Sensrad 2025-2026

#pragma once

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace kalman_filter {

// Type alias for 6x6 matrix used in SE(3) tangent space
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Kalman filter for poses on SE(3).
class PoseKalmanFilter {
 public:
  PoseKalmanFilter();

  // Initialize with an SE3 pose (uses default covariances)
  void init(const Sophus::SE3d& initial_state);

  // Predict using odometry delta
  void predict(const Sophus::SE3d& delta);

  // Update using a measured absolute pose
  void update(const Sophus::SE3d& measurement);

  // Accessors
  [[nodiscard]] Sophus::SE3d state() const { return state_; }
  [[nodiscard]] Matrix6d covariance() const { return covariance_; }

 private:
  Sophus::SE3d state_;   // State
  Matrix6d covariance_;  // Covariance in tangent space

  // Kalman filter parameters
  Matrix6d process_noise_;       // Process noise covariance
  Matrix6d measurement_noise_;   // Measurement noise covariance
  Matrix6d initial_covariance_;  // Initial covariance

  // Adjoint map of SE(3) for the 6-vector ordering
  static Matrix6d adjoint(const Sophus::SE3d& transform);

  // convert Sophus::SE3 log to 6-vector
  static Eigen::Matrix<double, 6, 1> se3Log(const Sophus::SE3d& transform);

  // Exponential map from 6-vector to SE3
  static Sophus::SE3d se3Exp(const Eigen::Matrix<double, 6, 1>& xi);

  static constexpr double Q_ANGLE_VARIANCE_DEFAULT = 1e-2;  // rad^2
  static constexpr double Q_POS_VARIANCE_XY = 0.1;          // m^2
  static constexpr double Q_POS_VARIANCE_Z = 0.3;           // m^2

  static constexpr double NOISE_SCALE_FACTOR = 15.0;

  static constexpr double P0_POS_VARIANCE_XY = 1000.0;  // m^2
  static constexpr double P0_POS_VARIANCE_Z = 1000.0;   // m^2
  static constexpr double P0_ANGLE_VARIANCE = 1.5;      // rad^2
};

}  // namespace kalman_filter
