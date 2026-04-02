// Copyright (c) Sensrad 2026
// Copyright (c) 2024 Saurabh Gupta, Tiziano Guadagnino, Benedikt Mersch,
// Ignacio Vizzo, Cyrill Stachniss.
// SPDX-License-Identifier: MIT

#pragma once

#include <vector>

#include <Eigen/Core>

namespace map_closures {
Eigen::Matrix4d alignToLocalGround(const std::vector<Eigen::Vector3d>& pointcloud,
                                   double resolution);

// Constants
static constexpr double CONVERGENCE_THRESHOLD = 1e-3;
static constexpr int MAX_ITERATIONS = 20;
}  // namespace map_closures
