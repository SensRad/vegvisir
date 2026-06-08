// Copyright (c) Sensrad 2026

// SVD-based point-to-point ICP for loop closure refinement.
// Uses MSE monitoring with early stop and separate convergence thresholds.

#include "icp_svd/IcpSvd.hpp"

namespace icp {

Eigen::Matrix4d IcpSvd::makeSE3(const Eigen::Matrix3d& rotation_matrix,
                                const Eigen::Vector3d& translation) {
  Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
  transform_matrix.block<3, 3>(0, 0) = rotation_matrix;
  transform_matrix.block<3, 1>(0, 3) = translation;
  return transform_matrix;
}

double IcpSvd::rotationAngle(const Eigen::Matrix3d& rotation_matrix) {
  const double cos_a = std::clamp((rotation_matrix.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cos_a);
}

std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> IcpSvd::computeAlignment(
    const Eigen::Vector3d *src, const Eigen::Vector3d *tgt, const double *dist_sq, size_t count,
    double cauchy_c_sq) {
  // Cauchy-robust weighted centroids
  double w_sum = 0.0;
  Eigen::Vector3d src_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d tgt_mean = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < count; ++i) {
    const double w = 1.0 / (1.0 + dist_sq[i] / cauchy_c_sq);
    src_mean += w * src[i];
    tgt_mean += w * tgt[i];
    w_sum += w;
  }
  src_mean /= w_sum;
  tgt_mean /= w_sum;

  // Weighted cross-covariance
  Eigen::Matrix3d cross_covariance = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < count; ++i) {
    const double w = 1.0 / (1.0 + dist_sq[i] / cauchy_c_sq);
    cross_covariance += w * (src[i] - src_mean) * (tgt[i] - tgt_mean).transpose();
  }

  // SVD + degeneracy check
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(cross_covariance,
                                              Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto& sigma = svd.singularValues();

  if (sigma(2) < 1e-6 * sigma(0)) {
    return std::nullopt;
  }

  // Reflection correction
  Eigen::Matrix3d svd_v = svd.matrixV();
  const Eigen::Matrix3d& svd_u = svd.matrixU();
#pragma GCC diagnostic pop
  if ((svd_v * svd_u.transpose()).determinant() < 0.0) {
    svd_v.col(2) *= -1.0;
  }

  const Eigen::Matrix3d delta_rotation = svd_v * svd_u.transpose();
  const Eigen::Vector3d delta_translation = tgt_mean - delta_rotation * src_mean;
  return std::pair{delta_rotation, delta_translation};
}

Result IcpSvd::pointToPointICP(const std::vector<Eigen::Vector3d>& source,
                               const std::vector<Eigen::Vector3d>& target,
                               const Eigen::Matrix4d& initial_guess, double voxel_size,
                               int max_iterations, double convergence_threshold,
                               double max_correspondence_distance) {
  voxel_map::VoxelMap target_map(voxel_size);
  target_map.addPoints(target);

  const double max_dist_sq = max_correspondence_distance * max_correspondence_distance;
  const double cauchy_c_sq = CAUCHY_SCALE * max_dist_sq;

  // Track rotation, translation separately; only build 4x4 at return
  Eigen::Matrix3d rotation_matrix = initial_guess.block<3, 3>(0, 0);
  Eigen::Vector3d translation = initial_guess.block<3, 1>(0, 3);
  Eigen::Matrix3d best_rotation = rotation_matrix;
  Eigen::Vector3d best_translation = translation;
  size_t best_num_corr = 0;
  double best_mse = std::numeric_limits<double>::max();
  double prev_mse = std::numeric_limits<double>::max();

  // Pre-allocate correspondence buffers
  const size_t n = source.size();
  std::vector<Eigen::Vector3d> src_world(n);
  std::vector<Eigen::Vector3d> tgt_matched(n);
  std::vector<double> dist_sq_arr(n);

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Data association
    size_t num_corr = 0;
    double mse = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const Eigen::Vector3d tp = rotation_matrix * source[i] + translation;
      auto [neighbor, dsq] = target_map.getClosestNeighbor(tp);
      if (dsq < max_dist_sq) {
        src_world[num_corr] = tp;
        tgt_matched[num_corr] = neighbor;
        dist_sq_arr[num_corr] = dsq;
        mse += dsq;
        ++num_corr;
      }
    }

    if (num_corr < MIN_CORRESPONDENCES) {
      return {makeSE3(best_rotation, best_translation), false, iter};
    }

    mse /= static_cast<double>(num_corr);

    // Fitness-aware best
    if (num_corr > best_num_corr || (num_corr == best_num_corr && mse < best_mse)) {
      best_num_corr = num_corr;
      best_mse = mse;
      best_rotation = rotation_matrix;
      best_translation = translation;
    }
    if (iter > 0 && mse > MSE_HYSTERESIS * prev_mse) {
      return {makeSE3(best_rotation, best_translation), true, iter};
    }
    prev_mse = mse;

    // ---- Align ----
    auto alignment = computeAlignment(src_world.data(), tgt_matched.data(), dist_sq_arr.data(),
                                      num_corr, cauchy_c_sq);
    if (!alignment) {
      return {makeSE3(best_rotation, best_translation), false, iter};
    }
    const auto& [delta_rotation, delta_translation] = *alignment;

    // ---- Compose ----
    rotation_matrix = delta_rotation * rotation_matrix;
    translation = delta_rotation * translation + delta_translation;

    // ---- Convergence check ----
    if (delta_translation.norm() < convergence_threshold &&
        rotationAngle(delta_rotation) < convergence_threshold * ROT_CONVERGENCE_SCALE) {
      return {makeSE3(rotation_matrix, translation), true, iter + 1};
    }
  }

  return {makeSE3(best_rotation, best_translation), false, max_iterations};
}

}  // namespace icp
