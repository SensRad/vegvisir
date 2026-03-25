// Copyright (c) Sensrad 2026
// Copyright (c) 2024 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
// SPDX-License-Identifier: MIT

#pragma once

#include <cmath>
#include <cstdint>

#include <vector>

#include <Eigen/Core>
#include <tsl/robin_map.h>

using Voxel = Eigen::Vector3i;

// Hash specialization for Voxel (Eigen::Vector3i).
// Must be defined before any inclusion of tsl/robin_map.h.
template <>
struct std::hash<Voxel> {
  std::size_t operator()(const Voxel& v) const {
    const auto *d = reinterpret_cast<const uint32_t *>(v.data());
    return d[0] * 73856093 ^ d[1] * 19349669 ^ d[2] * 83492791;
  }
};

namespace voxel_map {

inline Voxel pointToVoxel(const Eigen::Vector3d& point, double voxel_size) {
  return {static_cast<int>(std::floor(point.x() / voxel_size)),
          static_cast<int>(std::floor(point.y() / voxel_size)),
          static_cast<int>(std::floor(point.z() / voxel_size))};
}

// Voxel-grid downsample: keep one point per voxel (first-in wins).
std::vector<Eigen::Vector3d> voxelDownsample(const std::vector<Eigen::Vector3d>& points,
                                             double voxel_size);

}  // namespace voxel_map
