// Copyright (c) Sensrad 2026
// Copyright (c) 2025 Tiziano Guadagnino, Benedikt Mersch, Saurabh Gupta, Cyrill
// Stachniss.
// SPDX-License-Identifier: MIT
#include "VoxelMap.hpp"

#include <algorithm>
#include <numeric>

#include <Eigen/Eigenvalues>

namespace {

constexpr unsigned int MIN_POINTS_FOR_COVARIANCE_COMPUTATION = 3;

std::tuple<Eigen::Vector3d, Eigen::Vector3d> computeCentroidAndNormal(
    const voxel_map::VoxelBlock& coordinates) {
  const auto num_points = static_cast<double>(coordinates.size());
  const Eigen::Vector3d mean =
      std::reduce(coordinates.cbegin(), coordinates.cend(), Eigen::Vector3d().setZero()) /
      num_points;

  const Eigen::Matrix3d covariance =
      std::transform_reduce(coordinates.cbegin(), coordinates.cend(), Eigen::Matrix3d().setZero(),
                            std::plus<>(),
                            [&mean](const Eigen::Vector3d& point) {
                              const Eigen::Vector3d centered = point - mean;
                              return Eigen::Matrix3d(centered * centered.transpose());
                            }) /
      std::max(num_points - 1.0, 1.0);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  return {mean, solver.eigenvectors().col(0)};
}

}  // namespace

namespace voxel_map {

void VoxelBlock::emplaceBack(const Eigen::Vector3d& p) {
  if (size() < MAX_POINTS_PER_NORMAL_COMPUTATION) {
    points_.at(num_points_) = p;
    ++num_points_;
  }
}

VoxelMap::VoxelMap(double voxel_size)
    : voxel_size_(voxel_size),
      map_resolution_sq_(voxel_size * voxel_size / MAX_POINTS_PER_NORMAL_COMPUTATION) {}

std::vector<Eigen::Vector3d> VoxelMap::pointcloud() const {
  std::vector<Eigen::Vector3d> points;
  points.reserve(map_.size() * MAX_POINTS_PER_NORMAL_COMPUTATION);
  for (const auto& [_, block] : map_) {
    for (const auto& pt : block) {
      points.emplace_back(pt);
    }
  }
  points.shrink_to_fit();
  return points;
}

void VoxelMap::integrateFrame(const std::vector<Eigen::Vector3d>& points,
                              const Eigen::Matrix4d& pose) {
  std::vector<Eigen::Vector3d> points_transformed(points.size());
  const auto& r = pose.block<3, 3>(0, 0);
  const auto& t = pose.block<3, 1>(0, 3);
  std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
                 [&](const auto& point) { return r * point + t; });
  addPoints(points_transformed);
}

void VoxelMap::addPoints(const std::vector<Eigen::Vector3d>& points) {
  std::for_each(points.cbegin(), points.cend(), [&](const auto& point) {
    const auto voxel = pointToVoxel(point, voxel_size_);
    const auto& [it, inserted] = map_.insert({voxel, VoxelBlock()});
    if (!inserted) {
      auto& voxel_points = it.value();
      if (voxel_points.size() == MAX_POINTS_PER_NORMAL_COMPUTATION ||
          std::any_of(voxel_points.cbegin(), voxel_points.cend(), [&](const auto& voxel_point) {
            return (voxel_point - point).squaredNorm() < map_resolution_sq_;
          })) {
        return;
      }
    }
    it.value().emplaceBack(point);
  });
}

std::tuple<Vector3dVector, Vector3dVector> VoxelMap::perVoxelPointAndNormal() const {
  Vector3dVector points;
  points.reserve(map_.size());
  Vector3dVector normals;
  normals.reserve(map_.size());
  for (const auto& [_, block] : map_) {
    if (block.size() >= MIN_POINTS_FOR_COVARIANCE_COMPUTATION) {
      const auto& [mean, normal] = computeCentroidAndNormal(block);
      points.emplace_back(mean);
      normals.emplace_back(normal);
    }
  }
  points.shrink_to_fit();
  normals.shrink_to_fit();
  return {points, normals};
}

void VoxelMap::pruneFarPoints(const Eigen::Matrix4d& reference_pose, double max_distance) {
  std::vector<Voxel> voxels_to_remove;
  const Eigen::Vector3d reference_position = reference_pose.block<3, 1>(0, 3);
  const double max_distance_sq = max_distance * max_distance;
  for (const auto& [voxel, _] : map_) {
    const Eigen::Vector3d voxel_center =
        (voxel.cast<double>() + Eigen::Vector3d::Constant(0.5)) * voxel_size_;
    if ((voxel_center - reference_position).squaredNorm() > max_distance_sq) {
      voxels_to_remove.push_back(voxel);
    }
  }
  for (const auto& voxel : voxels_to_remove) {
    map_.erase(voxel);
  }
}

namespace {
constexpr double axisMinDistSq(int offset, double face_neg, double face_pos) {
  if (offset < 0)
    return face_neg * face_neg;
  if (offset > 0)
    return face_pos * face_pos;
  return 0.0;
}
}  // namespace

std::pair<Eigen::Vector3d, double> VoxelMap::getClosestNeighbor(
    const Eigen::Vector3d& query) const {
  const Eigen::Vector3d scaled = query / voxel_size_;
  const Voxel center = pointToVoxel(query, voxel_size_);

  Eigen::Vector3d best_point = Eigen::Vector3d::Zero();
  double best_dist_sq = std::numeric_limits<double>::max();

  auto update_best = [&](const VoxelBlock& block) {
    for (const auto& pt : block) {
      if (const double d2 = (query - pt).squaredNorm(); d2 < best_dist_sq) {
        best_dist_sq = d2;
        best_point = pt;
      }
    }
  };

  if (auto it = map_.find(center); it != map_.end()) {
    update_best(it->second);
  }

  // Distance from query to each face of the center voxel
  const Eigen::Vector3d frac = (scaled - center.cast<double>()) * voxel_size_;
  const Eigen::Vector3d face_pos = Eigen::Vector3d::Constant(voxel_size_) - frac;

  for (int dx = -1; dx <= 1; ++dx) {
    const double min_dx_sq = axisMinDistSq(dx, frac.x(), face_pos.x());
    if (min_dx_sq >= best_dist_sq) {
      continue;
    }

    for (int dy = -1; dy <= 1; ++dy) {
      const double min_dxy_sq = min_dx_sq + axisMinDistSq(dy, frac.y(), face_pos.y());
      if (min_dxy_sq >= best_dist_sq) {
        continue;
      }

      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }

        const double min_dist_sq = min_dxy_sq + axisMinDistSq(dz, frac.z(), face_pos.z());
        if (min_dist_sq >= best_dist_sq) {
          continue;
        }

        if (auto it = map_.find(center + Voxel(dx, dy, dz)); it != map_.end()) {
          update_best(it->second);
        }
      }
    }
  }
  return {best_point, best_dist_sq};
}

}  // namespace voxel_map
