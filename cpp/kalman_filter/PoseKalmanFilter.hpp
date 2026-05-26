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

  // Measurement noise parameters
  static constexpr double R_ANGLE_VARIANCE_ROLL = 0.01;   // rad^2
  static constexpr double R_ANGLE_VARIANCE_PITCH = 0.01;  // rad^2
  static constexpr double R_ANGLE_VARIANCE_YAW = 0.001;   // rad^2
  static constexpr double R_POS_VARIANCE_XY = 1.0;        // m^2
  static constexpr double R_POS_VARIANCE_Z = 1.0;         // m^2

  static constexpr double P0_POS_VARIANCE_XY = 1000.0;    // m^2
  static constexpr double P0_POS_VARIANCE_Z = 1000.0;     // m^2
  static constexpr double P0_ANGLE_VARIANCE_ROLL = 0.5;   // rad^2
  static constexpr double P0_ANGLE_VARIANCE_PITCH = 0.5;  // rad^2
  static constexpr double P0_ANGLE_VARIANCE_YAW = 0.05;   // rad^2

  // Process noise parameters
  static constexpr double Q_XY_PER_M = 1e-1;       // m^2 per meter
  static constexpr double Q_Z_PER_M = 1e-1;        // m^2 per meter
  static constexpr double Q_ROLL_PER_RAD = 1e-2;   // rad^2 per radian
  static constexpr double Q_PITCH_PER_RAD = 1e-2;  // rad^2 per radian
  static constexpr double Q_YAW_PER_RAD = 1e-3;    // rad^2 per radian
  static constexpr double Q_XY_PER_RAD = 1e-2;     // m^2 per radian
  static constexpr double Q_TIME_XY = 1e-4;        // m^2 per second
  static constexpr double Q_TIME_ROLL = 1e-6;      // rad^2 per second
  static constexpr double Q_TIME_PITCH = 1e-6;     // rad^2 per second
  static constexpr double Q_TIME_YAW = 1e-6;       // rad^2 per second
};

}  // namespace kalman_filter
