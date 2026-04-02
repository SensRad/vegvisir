// Copyright (c) Sensrad 2026
// Copyright (c) 2025 Tiziano Guadagnino, Benedikt Mersch, Saurabh Gupta, Cyrill
// Stachniss.
// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

#include <array>
#include <limits>
#include <tuple>
#include <vector>

#include <Eigen/Core>

// VoxelUtils.hpp provides Voxel type, hash, and pointToVoxel — must come
// before robin_map.h so the hash specialization is visible.
#include <tsl/robin_map.h>

#include "voxel_map/VoxelUtils.hpp"

using Vector3dVector = std::vector<Eigen::Vector3d>;

// Same default as Open3d
static constexpr unsigned int MAX_POINTS_PER_NORMAL_COMPUTATION = 20;

namespace voxel_map {

struct VoxelBlock {
  void emplaceBack(const Eigen::Vector3d& point);
  [[nodiscard]] constexpr size_t size() const { return num_points_; }
  [[nodiscard]] auto begin() const { return points_.cbegin(); }
  [[nodiscard]] auto end() const {
    return std::next(points_.cbegin(), static_cast<std::ptrdiff_t>(num_points_));
  }
  [[nodiscard]] auto cbegin() const { return points_.cbegin(); }
  [[nodiscard]] auto cend() const {
    return std::next(points_.cbegin(), static_cast<std::ptrdiff_t>(num_points_));
  }

 private:
  std::array<Eigen::Vector3d, MAX_POINTS_PER_NORMAL_COMPUTATION> points_;
  size_t num_points_ = 0;
};

struct VoxelMap {
  explicit VoxelMap(double voxel_size);

  void clear() { map_.clear(); }
  [[nodiscard]] bool empty() const { return map_.empty(); }
  void integrateFrame(const std::vector<Eigen::Vector3d>& points, const Eigen::Matrix4d& pose);
  void addPoints(const std::vector<Eigen::Vector3d>& points);
  [[nodiscard]] Vector3dVector pointcloud() const;

  void pruneFarPoints(const Eigen::Matrix4d& reference_pose, double max_distance);

  [[nodiscard]] size_t numVoxels() const { return map_.size(); }

  [[nodiscard]] std::tuple<Vector3dVector, Vector3dVector> perVoxelPointAndNormal() const;

  // Find the closest stored point to query using bounded neighbor search.
  // Checks center voxel first, then skips neighbor voxels whose bounding-box
  // minimum distance already exceeds the current best. Returns
  // {closest_point, squared_distance}.
  [[nodiscard]] std::pair<Eigen::Vector3d, double> getClosestNeighbor(
      const Eigen::Vector3d& query) const;

 private:
  double voxel_size_;
  double map_resolution_sq_;
  tsl::robin_map<Voxel, VoxelBlock> map_;
};
}  // namespace voxel_map
