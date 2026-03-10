// Copyright (c) Sensrad 2025-2026

#include "voxel_map/VoxelUtils.hpp"

namespace voxel_map {

std::vector<Eigen::Vector3d>
voxelDownsample(const std::vector<Eigen::Vector3d> &points, double voxel_size) {
  tsl::robin_map<Voxel, Eigen::Vector3d> grid;
  grid.reserve(points.size());
  for (const auto &p : points) {
    const auto voxel = pointToVoxel(p, voxel_size);
    if (!grid.contains(voxel)) {
      grid.insert({voxel, p});
    }
  }

  std::vector<Eigen::Vector3d> downsampled;
  downsampled.reserve(grid.size());
  for (const auto &[_, point] : grid) {
    downsampled.emplace_back(point);
  }
  return downsampled;
}

} // namespace voxel_map
