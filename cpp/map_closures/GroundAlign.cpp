// Copyright (c) Sensrad 2026
// Copyright (c) 2024 Saurabh Gupta, Tiziano Guadagnino, Benedikt Mersch,
// Ignacio Vizzo, Cyrill Stachniss.
// SPDX-License-Identifier: MIT

#include "GroundAlign.hpp"

#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace {
struct PixelHash {
  size_t operator()(const Eigen::Vector2i& pixel) const {
    const auto *vec = reinterpret_cast<const uint32_t *>(pixel.data());
    return (vec[0] * 73856093 ^ vec[1] * 19349669);
  }
};

void transformPoints(const Sophus::SE3d& t, std::vector<Eigen::Vector3d>& pointcloud) {
  std::transform(pointcloud.cbegin(), pointcloud.cend(), pointcloud.begin(),
                 [&](const auto& point) { return t * point; });
}

using LinearSystem = std::pair<Eigen::Matrix3d, Eigen::Vector3d>;

double computeAbsMedianZ(const std::vector<Eigen::Vector3d>& points) {
  if (points.empty()) {
    return 0.0;
  }
  std::vector<double> abs_z;
  abs_z.reserve(points.size());
  for (const auto& p : points) {
    abs_z.push_back(std::abs(p.z()));
  }
  const size_t mid = abs_z.size() / 2;
  std::nth_element(abs_z.begin(), abs_z.begin() + mid, abs_z.end());
  return abs_z[mid];
}

LinearSystem buildLinearSystem(const std::vector<Eigen::Vector3d>& points) {
  // Adaptive Tukey scale: c = K * 1.4826 * MAD(z), floored so we don't
  // over-tighten once the fit has converged on noise-level residuals.
  constexpr double MAD_TO_SIGMA = 1.4826;
  const double mad = computeAbsMedianZ(points);
  const double c =
      std::max(map_closures::TUKEY_K * MAD_TO_SIGMA * mad, map_closures::GROUND_INLIER_FLOOR_M);

  constexpr double inv_2sigma2_radial =
      1.0 / (2.0 * map_closures::GROUND_RADIAL_SIGMA_M * map_closures::GROUND_RADIAL_SIGMA_M);

  auto compute_jacobian_and_residual = [](const auto& point) {
    const double residual = point.z();
    Eigen::Matrix<double, 1, 3> j;
    j(0, 0) = 1.0;
    j(0, 1) = point.y();
    j(0, 2) = -point.x();
    return std::make_pair(j, residual);
  };

  auto sum_linear_systems = [](LinearSystem a, const LinearSystem& b) {
    a.first += b.first;
    a.second += b.second;
    return a;
  };

  const auto& [H, b] = std::transform_reduce(
      points.cbegin(), points.cend(),
      LinearSystem(Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero()), sum_linear_systems,
      [&](const auto& point) {
        const auto& [j, residual] = compute_jacobian_and_residual(point);

        // Tukey biweight on the z residual: smooth descending estimator
        // tightens the inlier set as the fit converges instead of leaning
        // on the loose `resolution`-sized window.
        const double r_over_c = residual / c;
        const double inlier = (1.0 - r_over_c * r_over_c);
        const double w_robust = (std::abs(r_over_c) < 1.0) ? inlier * inlier : 0.0;

        // Radial decay around the vehicle. Road returns under the lidar
        // dominate; lateral curb/wall/vegetation returns at the shoulder
        // cannot lever an asymmetric tilt onto the plane.
        const double r2 = point.x() * point.x() + point.y() * point.y();
        const double w_radial = std::exp(-r2 * inv_2sigma2_radial);

        const double w = w_robust * w_radial;
        return LinearSystem(j.transpose() * w * j,          // JTJ
                            j.transpose() * w * residual);  // JTr
      });
  return {H, b};
}

std::vector<Eigen::Vector3d> computeLowestPoints(const std::vector<Eigen::Vector3d>& pointcloud,
                                                 const double resolution) {
  std::unordered_map<Eigen::Vector2i, Eigen::Vector3d, PixelHash> lowest_point_hash_map;
  auto point_to_pixel = [&resolution](const Eigen::Vector3d& pt) -> Eigen::Vector2i {
    return {static_cast<int>(std::floor(pt.x() / resolution)),
            static_cast<int>(std::floor(pt.y() / resolution))};
  };

  std::for_each(pointcloud.cbegin(), pointcloud.cend(), [&](const Eigen::Vector3d& point) {
    const auto& pixel = point_to_pixel(point);
    if (lowest_point_hash_map.find(pixel) == lowest_point_hash_map.cend()) {
      lowest_point_hash_map.emplace(pixel, point);
    } else if (point.z() < lowest_point_hash_map[pixel].z()) {
      lowest_point_hash_map[pixel] = point;
    }
  });

  std::vector<Eigen::Vector3d> low_lying_points(lowest_point_hash_map.size());
  std::transform(lowest_point_hash_map.cbegin(), lowest_point_hash_map.cend(),
                 low_lying_points.begin(), [](const auto& entry) { return entry.second; });
  return low_lying_points;
}
}  // namespace

namespace map_closures {
Eigen::Matrix4d alignToLocalGround(const std::vector<Eigen::Vector3d>& pointcloud,
                                   const double resolution) {
  Sophus::SE3d t = Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  auto low_lying_points = computeLowestPoints(pointcloud, resolution);

  for (int iters = 0; iters < MAX_ITERATIONS; iters++) {
    const auto& [H, b] = buildLinearSystem(low_lying_points);
    const Eigen::Vector3d& dx = H.ldlt().solve(-b);
    Eigen::Matrix<double, 6, 1> se3 = Eigen::Matrix<double, 6, 1>::Zero();
    se3.block<3, 1>(2, 0) = dx;
    const Sophus::SE3d estimation(Sophus::SE3d::exp(se3));
    transformPoints(estimation, low_lying_points);
    t = estimation * t;
    if (dx.norm() < CONVERGENCE_THRESHOLD) {
      break;
    }
  }
  return t.matrix();
}
}  // namespace map_closures
