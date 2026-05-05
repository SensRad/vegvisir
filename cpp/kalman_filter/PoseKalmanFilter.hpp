// Copyright (c) Sensrad 2026

#pragma once

#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace kalman_filter {

// Type alias for 6x6 matrix used in SE(3) tangent space
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Process noise *rates* — variance accumulated per unit motion
struct ProcessNoiseRates {
  double sigma2_xy_per_m;     // (m²) of xy variance per meter traveled
  double sigma2_z_per_m;      // (m²) of z variance per meter traveled
  double sigma2_rot_per_rad;  // (rad²) of rotation variance per radian rotated
  double sigma2_xy_per_rad;   // cross-coupling: rotation induces xy drift
  double sigma2_time_xy;      // (m²/s) — small idle drift even when stationary
  double sigma2_time_rot;     // (rad²/s)
};

// Kalman filter for poses on SE(3).
class PoseKalmanFilter {
 public:
  PoseKalmanFilter();

  // Initialize with an SE3 pose (uses default covariances)
  void init(const Sophus::SE3d& initial_state);

  // Predict using odometry delta over elapsed time dt (seconds)
  void predict(const Sophus::SE3d& delta, double dt);

  // Update using a measured absolute pose
  void update(const Sophus::SE3d& measurement);

  // Accessors
  [[nodiscard]] Sophus::SE3d state() const { return state_; }
  [[nodiscard]] Matrix6d covariance() const { return covariance_; }

 private:
  Sophus::SE3d state_;   // State
  Matrix6d covariance_;  // Covariance in tangent space

  // Kalman filter parameters
  ProcessNoiseRates rates_;      // Process noise rates (motion- and time-scaled)
  Matrix6d measurement_noise_;   // Measurement noise covariance
  Matrix6d initial_covariance_;  // Initial covariance

  // Adjoint map of SE(3) for the 6-vector ordering
  static Matrix6d adjoint(const Sophus::SE3d& transform);

  // convert Sophus::SE3 log to 6-vector
  static Eigen::Matrix<double, 6, 1> se3Log(const Sophus::SE3d& transform);

  // Exponential map from 6-vector to SE3
  static Sophus::SE3d se3Exp(const Eigen::Matrix<double, 6, 1>& xi);

  static constexpr double R_ANGLE_VARIANCE = 0.15;  // rad^2
  static constexpr double R_POS_VARIANCE_XY = 1.5;  // m^2
  static constexpr double R_POS_VARIANCE_Z = 4.5;   // m^2

  static constexpr double P0_POS_VARIANCE_XY = 1000.0;  // m^2
  static constexpr double P0_POS_VARIANCE_Z = 1000.0;   // m^2
  static constexpr double P0_ANGLE_VARIANCE = 1.5;      // rad^2
};

}  // namespace kalman_filter
