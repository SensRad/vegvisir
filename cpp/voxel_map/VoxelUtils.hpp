// Copyright (c) Sensrad 2025-2026
//
// MIT License
//
// Copyright (c) 2024 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
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
//

// Voxel utilities: coordinate hashing, point-to-voxel conversion, and
// voxel-grid downsampling. Replaces the kiss_icp dependency for these
// primitives within the localization package.

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

inline Voxel pointToVoxel(const Eigen::Vector3d &point, double voxel_size) {
  return {static_cast<int>(std::floor(point.x() / voxel_size)),
          static_cast<int>(std::floor(point.y() / voxel_size)),
          static_cast<int>(std::floor(point.z() / voxel_size))};
}

// Voxel-grid downsample: keep one point per voxel (first-in wins).
std::vector<Eigen::Vector3d> voxelDownsample(const std::vector<Eigen::Vector3d>& points,
                                             double voxel_size);

}  // namespace voxel_map
