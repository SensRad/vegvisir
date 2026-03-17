// Copyright (c) Sensrad 2025-2026
//
// MIT License
//
// Copyright (c) 2025 Saurabh Gupta, Tiziano Guadagnino, Benedikt Mersch,
// Niklas Trekel, Meher Malladi, and Cyrill Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
  size_t operator()(const Eigen::Vector2i &pixel) const {
    const auto *vec = reinterpret_cast<const uint32_t *>(pixel.data());
    return (vec[0] * 73856093 ^ vec[1] * 19349669);
  }
};

void transformPoints(const Sophus::SE3d &t,
                     std::vector<Eigen::Vector3d> &pointcloud) {
  std::transform(pointcloud.cbegin(), pointcloud.cend(), pointcloud.begin(),
                 [&](const auto &point) { return t * point; });
}

using LinearSystem = std::pair<Eigen::Matrix3d, Eigen::Vector3d>;
LinearSystem buildLinearSystem(const std::vector<Eigen::Vector3d> &points,
                               const double resolution) {
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

  const auto &[H, b] = std::transform_reduce(
      points.cbegin(), points.cend(),
      LinearSystem(Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero()),
      sum_linear_systems, [&](const auto &point) {
        const auto &[j, residual] = compute_jacobian_and_residual(point);
        const double w = std::abs(residual) <= resolution ? 1.0 : 0.0;
        return LinearSystem(j.transpose() * w * j,         // JTJ
                            j.transpose() * w * residual); // JTr
      });
  return {H, b};
}

std::vector<Eigen::Vector3d>
computeLowestPoints(const std::vector<Eigen::Vector3d> &pointcloud,
                    const double resolution) {
  std::unordered_map<Eigen::Vector2i, Eigen::Vector3d, PixelHash>
      lowest_point_hash_map;
  auto point_to_pixel =
      [&resolution](const Eigen::Vector3d &pt) -> Eigen::Vector2i {
    return {static_cast<int>(std::floor(pt.x() / resolution)),
            static_cast<int>(std::floor(pt.y() / resolution))};
  };

  std::for_each(pointcloud.cbegin(), pointcloud.cend(),
                [&](const Eigen::Vector3d &point) {
                  const auto &pixel = point_to_pixel(point);
                  if (lowest_point_hash_map.find(pixel) ==
                      lowest_point_hash_map.cend()) {
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
Eigen::Matrix4d
alignToLocalGround(const std::vector<Eigen::Vector3d> &pointcloud,
                   const double resolution) {
  Sophus::SE3d t =
      Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  auto low_lying_points = computeLowestPoints(pointcloud, resolution);

  for (int iters = 0; iters < MAX_ITERATIONS; iters++) {
    const auto &[H, b] = buildLinearSystem(low_lying_points, resolution);
    const Eigen::Vector3d &dx = H.ldlt().solve(-b);
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
