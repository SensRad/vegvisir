// Copyright (c) Sensrad 2025-2026
//
// SVD-based point-to-point ICP for loop closure refinement.
// Uses correspondence trimming, MSE monitoring with early stop, and
// separate convergence thresholds.

#include "icp_svd/IcpSvd.hpp"

namespace icp {

Eigen::Matrix4d IcpSvd::makeSE3(const Eigen::Matrix3d &R,
                                const Eigen::Vector3d &t) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

double IcpSvd::rotationAngle(const Eigen::Matrix3d &R) {
  const double cos_a = std::clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cos_a);
}

size_t IcpSvd::trimCorrespondences(uint32_t *indices, const double *dist_sq,
                                   size_t num_corr) {
  const size_t keep =
      std::max(MIN_CORRESPONDENCES,
               static_cast<size_t>(static_cast<double>(num_corr) * TRIM_RATIO));

  if (keep < num_corr) {
    std::nth_element(
        indices, indices + keep, indices + num_corr,
        [dist_sq](uint32_t a, uint32_t b) { return dist_sq[a] < dist_sq[b]; });
    return keep;
  }
  return num_corr;
}

std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>
IcpSvd::computeAlignment(const Eigen::Vector3d *src, const Eigen::Vector3d *tgt,
                         const double *dist_sq, const uint32_t *indices,
                         size_t count, double cauchy_c_sq) {
  // Cauchy-robust weighted centroids
  double w_sum = 0.0;
  Eigen::Vector3d src_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d tgt_mean = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < count; ++i) {
    const uint32_t idx = indices[i];
    const double w = 1.0 / (1.0 + dist_sq[idx] / cauchy_c_sq);
    src_mean += w * src[idx];
    tgt_mean += w * tgt[idx];
    w_sum += w;
  }
  src_mean /= w_sum;
  tgt_mean /= w_sum;

  // Weighted cross-covariance
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < count; ++i) {
    const uint32_t idx = indices[i];
    const double w = 1.0 / (1.0 + dist_sq[idx] / cauchy_c_sq);
    H += w * (src[idx] - src_mean) * (tgt[idx] - tgt_mean).transpose();
  }

  // SVD + degeneracy check
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  const auto &sigma = svd.singularValues();

  if (sigma(2) < 1e-6 * sigma(0)) {
    return std::nullopt;
  }

  // Reflection correction
  Eigen::Matrix3d V = svd.matrixV();
  const Eigen::Matrix3d &U = svd.matrixU();
#pragma GCC diagnostic pop
  if ((V * U.transpose()).determinant() < 0.0) {
    V.col(2) *= -1.0;
  }

  const Eigen::Matrix3d dR = V * U.transpose();
  const Eigen::Vector3d dt = tgt_mean - dR * src_mean;
  return std::pair{dR, dt};
}

Result IcpSvd::pointToPointICP(const std::vector<Eigen::Vector3d> &source,
                               const std::vector<Eigen::Vector3d> &target,
                               const Eigen::Matrix4d &initial_guess,
                               double voxel_size, int max_iterations,
                               double convergence_threshold,
                               double max_correspondence_distance) {

  voxel_map::VoxelMap target_map(voxel_size);
  target_map.AddPoints(target);

  const double max_dist_sq =
      max_correspondence_distance * max_correspondence_distance;
  const double cauchy_c_sq = 0.25 * max_dist_sq;

  // Track R, t separately; only build 4x4 at return
  Eigen::Matrix3d R = initial_guess.block<3, 3>(0, 0);
  Eigen::Vector3d t = initial_guess.block<3, 1>(0, 3);
  Eigen::Matrix3d best_R = R;
  Eigen::Vector3d best_t = t;
  double best_mse = std::numeric_limits<double>::max();
  double prev_mse = std::numeric_limits<double>::max();

  // Pre-allocate correspondence buffers
  const size_t n = source.size();
  std::vector<Eigen::Vector3d> src_world(n);
  std::vector<Eigen::Vector3d> tgt_matched(n);
  std::vector<double> dist_sq_arr(n);
  std::vector<uint32_t> indices(n);

  for (int iter = 0; iter < max_iterations; ++iter) {
    // ---- Data association ----
    size_t num_corr = 0;
    double mse = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const Eigen::Vector3d tp = R * source[i] + t;
      auto [neighbor, dsq] = target_map.GetClosestNeighbor(tp);
      if (dsq < max_dist_sq) {
        src_world[num_corr] = tp;
        tgt_matched[num_corr] = neighbor;
        dist_sq_arr[num_corr] = dsq;
        mse += dsq;
        ++num_corr;
      }
    }

    if (num_corr < MIN_CORRESPONDENCES) {
      return {makeSE3(best_R, best_t), false, iter};
    }

    mse /= static_cast<double>(num_corr);

    if (mse < best_mse) {
      best_mse = mse;
      best_R = R;
      best_t = t;
    }
    if (iter > 0 && mse > prev_mse) {
      return {makeSE3(best_R, best_t), true, iter};
    }
    prev_mse = mse;

    // ---- Trim + align ----
    std::iota(indices.begin(), indices.begin() + num_corr, uint32_t(0));
    const size_t kept =
        trimCorrespondences(indices.data(), dist_sq_arr.data(), num_corr);

    auto alignment =
        computeAlignment(src_world.data(), tgt_matched.data(),
                         dist_sq_arr.data(), indices.data(), kept, cauchy_c_sq);
    if (!alignment) {
      return {makeSE3(best_R, best_t), false, iter};
    }
    const auto &[dR, dt] = *alignment;

    // ---- Compose ----
    R = dR * R;
    t = dR * t + dt;

    // ---- Convergence check ----
    if (dt.norm() < convergence_threshold &&
        rotationAngle(dR) < convergence_threshold * ROT_CONVERGENCE_SCALE) {
      return {makeSE3(R, t), true, iter + 1};
    }
  }

  return {makeSE3(best_R, best_t), false, max_iterations};
}

} // namespace icp
