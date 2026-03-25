// Copyright (c) Sensrad 2026
// Copyright (c) 2024 Saurabh Gupta, Tiziano Guadagnino, Benedikt Mersch,
// Ignacio Vizzo, Cyrill Stachniss.
// SPDX-License-Identifier: MIT

#pragma once

#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace map_closures {

struct DensityMap {
  DensityMap() = default;
  DensityMap(int num_rows, int num_cols, double resolution, Eigen::Vector2i lower_bound);
  ~DensityMap() = default;
  DensityMap(const DensityMap& other) = delete;
  DensityMap(DensityMap&& other) = default;
  DensityMap& operator=(DensityMap&& other) = default;
  DensityMap& operator=(const DensityMap& other) = delete;
  auto& operator()(const int x, const int y) { return grid.at<uint8_t>(x, y); }
  Eigen::Vector2i lower_bound;
  double resolution = 0.0;
  cv::Mat grid;
};

DensityMap generateDensityMap(const std::vector<Eigen::Vector3d>& pcd,
                              const Eigen::Matrix4d& ground_transform, float density_map_resolution,
                              float density_threshold);

/// Apply gamma correction to density map for improved feature detection
/// Gamma < 1.0 brightens the image, > 1.0 darkens it, 1.0 = no change
void applyGammaCorrection(DensityMap& density_map, float gamma);
}  // namespace map_closures
