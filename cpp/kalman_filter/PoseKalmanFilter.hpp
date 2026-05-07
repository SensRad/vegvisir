// Copyright (c) Sensrad 2026

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
  Matrix6d measurement_noise_;   // Measurement noise covariance
  Matrix6d initial_covariance_;  // Initial covariance

  // convert Sophus::SE3 log to 6-vector
  static Eigen::Matrix<double, 6, 1> se3Log(const Sophus::SE3d& transform);

  // Exponential map from 6-vector to SE3
  static Sophus::SE3d se3Exp(const Eigen::Matrix<double, 6, 1>& xi);

  // Rotation axes are body-frame: x = roll, y = pitch, z = yaw. Yaw is much
  // better observed by the closure pipeline (2D RANSAC + ICP) than roll/pitch,
  // so its variance is correspondingly tighter. Roll/pitch share a value
  // because they have similar observability from a near-horizontal LiDAR.
  static constexpr double R_ANGLE_VARIANCE_ROLL = 0.01;   // (~5.7°)²
  static constexpr double R_ANGLE_VARIANCE_PITCH = 0.01;  // (~5.7°)²
  static constexpr double R_ANGLE_VARIANCE_YAW = 0.001;   // (~1.8°)²
  static constexpr double R_POS_VARIANCE_XY = 6.0;        // m^2
  static constexpr double R_POS_VARIANCE_Z = 15.0;        // m^2

  static constexpr double P0_POS_VARIANCE_XY = 1000.0;    // m^2
  static constexpr double P0_POS_VARIANCE_Z = 1000.0;     // m^2
  static constexpr double P0_ANGLE_VARIANCE_ROLL = 0.5;   // (~40°)²
  static constexpr double P0_ANGLE_VARIANCE_PITCH = 0.5;  // (~40°)²
  static constexpr double P0_ANGLE_VARIANCE_YAW = 0.05;   // (~13°)² — yaw less unknown at boot

  // Process-noise rates — variance accumulated per unit motion (or per second
  // when idle). Tuned for slow growth between closures so the filter holds a
  // stable map→odom estimate. Yaw rates are an order of magnitude tighter than
  // roll/pitch, mirroring the measurement-noise split.
  static constexpr double Q_XY_PER_M = 1e-3;       // m² per meter (≈3 cm σ/m)
  static constexpr double Q_Z_PER_M = 1e-3;        // m² per meter
  static constexpr double Q_ROLL_PER_RAD = 1e-3;   // rad² per radian
  static constexpr double Q_PITCH_PER_RAD = 1e-3;  // rad² per radian
  static constexpr double Q_YAW_PER_RAD = 1e-4;    // rad² per radian
  static constexpr double Q_XY_PER_RAD = 1e-3;     // m² per radian (cross-coupling)
  static constexpr double Q_TIME_XY = 1e-5;        // m² per second (idle)
  static constexpr double Q_TIME_ROLL = 1e-7;      // rad² per second (idle)
  static constexpr double Q_TIME_PITCH = 1e-7;     // rad² per second (idle)
  static constexpr double Q_TIME_YAW = 1e-7;       // rad² per second (idle)
};

}  // namespace kalman_filter
