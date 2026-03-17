// Copyright (c) Sensrad 2025-2026

#include "PoseKalmanFilter.hpp"

namespace vegvisir {

PoseKalmanFilter::PoseKalmanFilter() {
  state_ = Sophus::SE3d(); // identity
  covariance_.setZero();

  // Default process noise
  Eigen::Vector3d position_variance(Q_POS_VARIANCE_XY, Q_POS_VARIANCE_XY,
                                    Q_POS_VARIANCE_Z);

  process_noise_ = Matrix6d::Zero();
  process_noise_.block<3, 3>(0, 0) =
      Eigen::Matrix3d::Identity() * Q_ANGLE_VARIANCE_DEFAULT; // rotation
  process_noise_.block<3, 3>(3, 3) = position_variance.asDiagonal();      // translation

  // Measurement noise
  measurement_noise_ = process_noise_ * NOISE_SCALE_FACTOR;

  initial_covariance_ = Matrix6d::Zero();
  initial_covariance_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * P0_ANGLE_VARIANCE;
  initial_covariance_.block<3, 3>(3, 3) =
      (Eigen::Vector3d(P0_POS_VARIANCE_XY, P0_POS_VARIANCE_XY,
                       P0_POS_VARIANCE_Z))
          .matrix()
          .asDiagonal();
}

void PoseKalmanFilter::init(const Sophus::SE3d &initial_state) {
  state_ = initial_state;
  covariance_ = initial_covariance_; // Use default initial covariance
}

Matrix6d PoseKalmanFilter::adjoint(const Sophus::SE3d &transform) {
  Eigen::Matrix3d R = transform.rotationMatrix();
  Eigen::Vector3d t = transform.translation();

  Matrix6d Ad;
  Ad.setZero();
  Ad.block<3, 3>(0, 0) = R;
  Ad.block<3, 3>(3, 3) = R;

  // skew-symmetric of t
  Eigen::Matrix3d t_skew;
  t_skew << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;

  Ad.block<3, 3>(3, 0) = t_skew * R;
  return Ad;
}

Eigen::Matrix<double, 6, 1> PoseKalmanFilter::se3Log(const Sophus::SE3d &transform) {
  // Sophus::SE3d::log()
  Eigen::Matrix<double, 6, 1> xi = transform.log();
  return xi;
}

Sophus::SE3d PoseKalmanFilter::se3Exp(const Eigen::Matrix<double, 6, 1> &xi) {
  return Sophus::SE3d::exp(xi);
}

void PoseKalmanFilter::predict(const Sophus::SE3d &delta) {
  // For map->odom tracking: the transform is static between measurements
  // Robot motion in odom frame doesn't change the map->odom relationship
  // However, uncertainty grows proportionally to motion (odometry drift)

  // Compute motion magnitude
  double translation_dist = delta.translation().norm(); // meters
  double rotation_angle = delta.so3().log().norm();     // radians

  // Scale process noise by motion magnitude
  double motion_scale = std::max(translation_dist, rotation_angle);

  // Add scaled process noise
  double scale = std::max(motion_scale, 0.01);
  covariance_ = covariance_ + process_noise_ * scale;

  // state_ remains unchanged - map->odom is constant until next measurement
}

void PoseKalmanFilter::update(const Sophus::SE3d &measurement) {
  // Innovation: y = Log( X^{-1} * Z )
  const Sophus::SE3d XinvZ = state_.inverse() * measurement;
  const Eigen::Matrix<double, 6, 1> y = se3Log(XinvZ);

  // Innovation covariance S = P + R (H = I)
  Matrix6d S = covariance_ + measurement_noise_;

  // Guard against tiny numerical asymmetry
  S = 0.5 * (S + S.transpose());

  // Kalman gain without explicit inverse: K = P * S^{-1}
  const Matrix6d I6 = Matrix6d::Identity();

  Eigen::LDLT<Matrix6d> ldlt(S);

  const Matrix6d K = covariance_ * ldlt.solve(I6);

  // Update mean
  const Eigen::Matrix<double, 6, 1> dx = K * y;
  const Sophus::SE3d dX = se3Exp(dx);
  state_ = state_ * dX;

  // Joseph form covariance update
  const Matrix6d I = Matrix6d::Identity();

  covariance_ = (I - K) * covariance_ * (I - K).transpose() + K * measurement_noise_ * K.transpose();

  // Force symmetric
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

} // namespace vegvisir
