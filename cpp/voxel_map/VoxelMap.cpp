// Copyright (c) Sensrad 2025-2026
//
// MIT License

// Copyright (c) 2025 Tiziano Guadagnino, Benedikt Mersch, Saurabh Gupta, Cyrill
// Stachniss.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "VoxelMap.hpp"

#include <cmath>

#include <algorithm>
#include <numeric>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace {

static constexpr unsigned int min_points_for_covariance_computation = 3;

std::tuple<Eigen::Vector3d, Eigen::Vector3d> ComputeCentroidAndNormal(
    const voxel_map::VoxelBlock& coordinates) {
  const double num_points = static_cast<double>(coordinates.size());
  const Eigen::Vector3d& mean =
      std::reduce(coordinates.cbegin(), coordinates.cend(), Eigen::Vector3d().setZero()) /
      num_points;

  const Eigen::Matrix3d& covariance =
      std::transform_reduce(coordinates.cbegin(), coordinates.cend(), Eigen::Matrix3d().setZero(),
                            std::plus<Eigen::Matrix3d>(),
                            [&mean](const Eigen::Vector3d& point) {
                              const Eigen::Vector3d& centered = point - mean;
                              const Eigen::Matrix3d S = centered * centered.transpose();
                              return S;
                            }) /
      std::max((num_points - 1.0), 1.0);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  const Eigen::Vector3d normal = solver.eigenvectors().col(0);
  return std::make_tuple(mean, normal);
}

}  // namespace

namespace voxel_map {

void VoxelBlock::emplace_back(const Eigen::Vector3d& p) {
  if (size() < max_points_per_normal_computation) {
    points.at(num_points) = p;
    ++num_points;
  }
}

VoxelMap::VoxelMap(const double voxel_size)
    : voxel_size_(voxel_size),
      map_resolution_(voxel_size /
                      static_cast<double>(std::sqrt(max_points_per_normal_computation))) {}

std::vector<Eigen::Vector3d> VoxelMap::Pointcloud() const {
  std::vector<Eigen::Vector3d> points;
  points.reserve(map_.size() * max_points_per_normal_computation);
  std::for_each(map_.cbegin(), map_.cend(), [&](const auto& map_element) {
    const auto& voxel_points = map_element.second;
    std::for_each(voxel_points.cbegin(), voxel_points.cend(),
                  [&](const auto& p) { points.emplace_back(p); });
  });
  points.shrink_to_fit();
  return points;
}

void VoxelMap::IntegrateFrame(const std::vector<Eigen::Vector3d>& points,
                              const Eigen::Matrix4d& pose) {
  std::vector<Eigen::Vector3d> points_transformed(points.size());
  const auto& R = pose.block<3, 3>(0, 0);
  const auto& t = pose.block<3, 1>(0, 3);
  std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                 [&](const auto& point) { return R * point + t; });
  AddPoints(points_transformed);
}

void VoxelMap::AddPoints(const std::vector<Eigen::Vector3d>& points) {
  std::for_each(points.cbegin(), points.cend(), [&](const auto& point) {
    const auto voxel = pointToVoxel(point, voxel_size_);
    const auto& [it, inserted] = map_.insert({voxel, VoxelBlock()});
    if (!inserted) {
      auto& voxel_points = it.value();
      if (voxel_points.size() == max_points_per_normal_computation ||
          std::any_of(voxel_points.cbegin(), voxel_points.cend(), [&](const auto& voxel_point) {
            return (voxel_point - point).norm() < map_resolution_;
          })) {
        return;
      }
    }
    it.value().emplace_back(point);
  });
}

std::tuple<Vector3dVector, Vector3dVector> VoxelMap::PerVoxelPointAndNormal() const {
  Vector3dVector points;
  points.reserve(map_.size());
  Vector3dVector normals;
  normals.reserve(map_.size());
  std::for_each(map_.cbegin(), map_.cend(), [&](const auto& inner_block) {
    const auto& voxel_block = inner_block.second;
    if (voxel_block.size() >= min_points_for_covariance_computation) {
      const auto& [mean, normal] = ComputeCentroidAndNormal(voxel_block);
      points.emplace_back(mean);
      normals.emplace_back(normal);
    }
  });
  points.shrink_to_fit();
  normals.shrink_to_fit();
  return std::make_tuple(points, normals);
}

void VoxelMap::PruneFarPoints(const Eigen::Matrix4d& reference_pose, double max_distance) {
  std::vector<Voxel> voxels_to_remove;
  Eigen::Vector3d reference_position = reference_pose.block<3, 1>(0, 3);
  for (const auto& map_element : map_) {
    const Voxel& voxel = map_element.first;
    Eigen::Vector3d voxel_center =
        Eigen::Vector3d((static_cast<double>(voxel.x()) + 0.5) * voxel_size_,
                        (static_cast<double>(voxel.y()) + 0.5) * voxel_size_,
                        (static_cast<double>(voxel.z()) + 0.5) * voxel_size_);
    double distance = (voxel_center - reference_position).norm();
    if (distance > max_distance) {
      voxels_to_remove.push_back(voxel);
    }
  }
  for (const auto& voxel : voxels_to_remove) {
    map_.erase(voxel);
  }
}

std::pair<Eigen::Vector3d, double> VoxelMap::GetClosestNeighbor(
    const Eigen::Vector3d& query) const {
  const double qx = query.x() / voxel_size_;
  const double qy = query.y() / voxel_size_;
  const double qz = query.z() / voxel_size_;
  const int cx = static_cast<int>(std::floor(qx));
  const int cy = static_cast<int>(std::floor(qy));
  const int cz = static_cast<int>(std::floor(qz));

  Eigen::Vector3d best_point = Eigen::Vector3d::Zero();
  double best_dist_sq = std::numeric_limits<double>::max();

  // Helper: scan all points in a voxel block
  auto scan_block = [&](const VoxelBlock& block) {
    for (auto pit = block.cbegin(); pit != block.cend(); ++pit) {
      const double d2 = (query - *pit).squaredNorm();
      if (d2 < best_dist_sq) {
        best_dist_sq = d2;
        best_point = *pit;
      }
    }
  };

  // Check center voxel first to establish a tight initial bound
  if (auto it = map_.find(Voxel(cx, cy, cz)); it != map_.end()) {
    scan_block(it->second);
  }

  // Fractional position within center voxel (0..1), scaled to metric
  const double fx = (qx - cx) * voxel_size_;
  const double fy = (qy - cy) * voxel_size_;
  const double fz = (qz - cz) * voxel_size_;
  // Distance from query to each face of the center voxel
  const double dx_neg = fx;                // distance to x-low face
  const double dx_pos = voxel_size_ - fx;  // distance to x-high face
  const double dy_neg = fy;
  const double dy_pos = voxel_size_ - fy;
  const double dz_neg = fz;
  const double dz_pos = voxel_size_ - fz;

  // Minimum distance along one axis to the neighbor voxel:
  // d<0 → query-to-low-face, d>0 → query-to-high-face, d==0 → 0 (same voxel)
  auto axis_min = [](int d, double neg, double pos) -> double {
    return (d < 0) ? neg : (d > 0) ? pos : 0.0;
  };

  // Check 26 neighbors, skipping those whose bounding box is too far
  for (int dx = -1; dx <= 1; ++dx) {
    const double min_dx_sq = std::pow(axis_min(dx, dx_neg, dx_pos), 2);
    if (min_dx_sq >= best_dist_sq) {
      continue;
    }

    for (int dy = -1; dy <= 1; ++dy) {
      const double min_dxy_sq = min_dx_sq + std::pow(axis_min(dy, dy_neg, dy_pos), 2);
      if (min_dxy_sq >= best_dist_sq) {
        continue;
      }

      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;  // already checked
        }
        const double min_dist_sq = min_dxy_sq + std::pow(axis_min(dz, dz_neg, dz_pos), 2);
        if (min_dist_sq >= best_dist_sq) {
          continue;
        }

        const Voxel v(cx + dx, cy + dy, cz + dz);
        auto it = map_.find(v);
        if (it == map_.end()) {
          continue;
        }
        scan_block(it->second);
      }
    }
  }
  return {best_point, best_dist_sq};
}

}  // namespace voxel_map
