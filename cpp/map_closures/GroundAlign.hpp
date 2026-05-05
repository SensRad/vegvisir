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
static constexpr double CONVERGENCE_THRESHOLD = 1e-4;
static constexpr int MAX_ITERATIONS = 30;

// Robust-fit knobs used by buildLinearSystem.
static constexpr double GROUND_INLIER_FLOOR_M = 0.15;  // hard floor on Tukey scale
static constexpr double TUKEY_K = 4.685;               // standard Tukey biweight constant
static constexpr double GROUND_RADIAL_SIGMA_M = 6.0;   // vehicle-centered weight std-dev
}  // namespace map_closures
