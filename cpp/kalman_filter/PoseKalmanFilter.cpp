// Copyright (c) Sensrad 2026

#include "kalman_filter/PoseKalmanFilter.hpp"

namespace kalman_filter {

PoseKalmanFilter::PoseKalmanFilter() {
  state_ = Sophus::SE3d();  // identity
  covariance_.setZero();

  // Measurement noise
  measurement_noise_ = Matrix6d::Zero();
  measurement_noise_.block<3, 3>(0, 0) =
      Eigen::Vector3d(R_POS_VARIANCE_XY, R_POS_VARIANCE_XY, R_POS_VARIANCE_Z).asDiagonal();
  measurement_noise_.block<3, 3>(3, 3) =
      Eigen::Vector3d(R_ANGLE_VARIANCE_ROLL, R_ANGLE_VARIANCE_PITCH, R_ANGLE_VARIANCE_YAW)
          .asDiagonal();

  // Initial covariance
  initial_covariance_ = Matrix6d::Zero();
  initial_covariance_.block<3, 3>(0, 0) =
      Eigen::Vector3d(P0_POS_VARIANCE_XY, P0_POS_VARIANCE_XY, P0_POS_VARIANCE_Z).asDiagonal();
  initial_covariance_.block<3, 3>(3, 3) =
      Eigen::Vector3d(P0_ANGLE_VARIANCE_ROLL, P0_ANGLE_VARIANCE_PITCH, P0_ANGLE_VARIANCE_YAW)
          .asDiagonal();
}

void PoseKalmanFilter::init(const Sophus::SE3d& initial_state) {
  state_ = initial_state;
  covariance_ = initial_covariance_;  // Use default initial covariance
}

Eigen::Matrix<double, 6, 1> PoseKalmanFilter::se3Log(const Sophus::SE3d& transform) {
  // Sophus::SE3d::log()
  Eigen::Matrix<double, 6, 1> xi = transform.log();
  return xi;
}

Sophus::SE3d PoseKalmanFilter::se3Exp(const Eigen::Matrix<double, 6, 1>& xi) {
  return Sophus::SE3d::exp(xi);
}

void PoseKalmanFilter::predict(const Sophus::SE3d& delta, double dt) {
  // For map->odom tracking: the transform is static between measurements,
  // but uncertainty grows with motion (odometry drift) and a small idle term.
  const double trans_dist = delta.translation().norm();
  const double rot_angle = delta.so3().log().norm();

  // [upsilon (translation), omega (rotation)] block layout — see constructor.
  Matrix6d q = Matrix6d::Zero();

  const Eigen::Vector3d t_var(Q_XY_PER_M, Q_XY_PER_M, Q_Z_PER_M);
  q.block<3, 3>(0, 0) = Eigen::Matrix3d((t_var * trans_dist).asDiagonal()) +
                        Eigen::Matrix3d::Identity() * (Q_XY_PER_RAD * rot_angle + Q_TIME_XY * dt);

  const Eigen::Vector3d rot_var(Q_ROLL_PER_RAD * rot_angle + Q_TIME_ROLL * dt,
                                Q_PITCH_PER_RAD * rot_angle + Q_TIME_PITCH * dt,
                                Q_YAW_PER_RAD * rot_angle + Q_TIME_YAW * dt);
  q.block<3, 3>(3, 3) = rot_var.asDiagonal();

  covariance_ += q;

  // state_ remains unchanged - map->odom is constant until next measurement
}

void PoseKalmanFilter::update(const Sophus::SE3d& measurement) {
  // Innovation: y = Log( X^{-1} * Z )
  const Sophus::SE3d xinv_z = state_.inverse() * measurement;
  const Eigen::Matrix<double, 6, 1> y = se3Log(xinv_z);

  // Innovation covariance S = P + R (H = I)
  Matrix6d s = covariance_ + measurement_noise_;

  // Guard against numerical asymmetry
  s = 0.5 * (s + s.transpose());

  // Kalman gain without explicit inverse: K = P * S^{-1}
  const Matrix6d i6 = Matrix6d::Identity();

  const Eigen::LDLT<Matrix6d> ldlt(s);

  const Matrix6d k = covariance_ * ldlt.solve(i6);

  // Update mean
  const Eigen::Matrix<double, 6, 1> dx = k * y;
  const Sophus::SE3d d_x = se3Exp(dx);
  state_ = state_ * d_x;

  // Joseph form covariance update
  const Matrix6d i = Matrix6d::Identity();

  covariance_ =
      (i - k) * covariance_ * (i - k).transpose() + k * measurement_noise_ * k.transpose();

  // Force symmetric
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

}  // namespace kalman_filter
