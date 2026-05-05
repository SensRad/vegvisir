// Copyright (c) Sensrad 2026

#include "kalman_filter/PoseKalmanFilter.hpp"

namespace kalman_filter {

PoseKalmanFilter::PoseKalmanFilter() {
  state_ = Sophus::SE3d();  // identity
  covariance_.setZero();

  rates_ = ProcessNoiseRates{
      .sigma2_xy_per_m = 0.1,
      .sigma2_z_per_m = 0.1,
      .sigma2_rot_per_rad = 1e-2,
      .sigma2_xy_per_rad = 0.05,
      .sigma2_time_xy = 1e-3,
      .sigma2_time_rot = 1e-4,
  };

  measurement_noise_ = Matrix6d::Zero();
  measurement_noise_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * R_ANGLE_VARIANCE;
  measurement_noise_.block<3, 3>(3, 3) =
      Eigen::Vector3d(R_POS_VARIANCE_XY, R_POS_VARIANCE_XY, R_POS_VARIANCE_Z).asDiagonal();

  initial_covariance_ = Matrix6d::Zero();
  initial_covariance_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * P0_ANGLE_VARIANCE;
  initial_covariance_.block<3, 3>(3, 3) =
      Eigen::Vector3d(P0_POS_VARIANCE_XY, P0_POS_VARIANCE_XY, P0_POS_VARIANCE_Z).asDiagonal();
}

void PoseKalmanFilter::init(const Sophus::SE3d& initial_state) {
  state_ = initial_state;
  covariance_ = initial_covariance_;  // Use default initial covariance
}

Matrix6d PoseKalmanFilter::adjoint(const Sophus::SE3d& transform) {
  const Eigen::Matrix3d r = transform.rotationMatrix();
  Eigen::Vector3d t = transform.translation();

  Matrix6d ad;
  ad.setZero();
  ad.block<3, 3>(0, 0) = r;
  ad.block<3, 3>(3, 3) = r;

  // skew-symmetric of t
  Eigen::Matrix3d t_skew;
  t_skew << 0.0, -t.z(), t.y(), t.z(), 0.0, -t.x(), -t.y(), t.x(), 0.0;

  ad.block<3, 3>(3, 0) = t_skew * r;
  return ad;
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

  Matrix6d q = Matrix6d::Zero();
  q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() *
                        (rates_.sigma2_rot_per_rad * rot_angle + rates_.sigma2_time_rot * dt);

  const Eigen::Vector3d t_var(rates_.sigma2_xy_per_m, rates_.sigma2_xy_per_m,
                              rates_.sigma2_z_per_m);
  q.block<3, 3>(3, 3) = Eigen::Matrix3d((t_var * trans_dist).asDiagonal()) +
                        Eigen::Matrix3d::Identity() *
                            (rates_.sigma2_xy_per_rad * rot_angle + rates_.sigma2_time_xy * dt);

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
