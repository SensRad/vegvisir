// Copyright (c) Sensrad 2026
// Copyright (c) 2024 Saurabh Gupta, Tiziano Guadagnino, Benedikt Mersch,
// Ignacio Vizzo, Cyrill Stachniss.
// SPDX-License-Identifier: MIT

#pragma once

#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace map_closures {

static constexpr double RANSAC_INLIERS_RATIO = 0.3;
static constexpr double RANSAC_PROBABILITY_SUCCESS = 0.999;
static constexpr int RANSAC_MIN_POINTS = 2;
static constexpr int RANSAC_TRIALS = 200;

struct PointPair {
  PointPair() = default;
  PointPair(Eigen::Vector2d r, Eigen::Vector2d q) : ref(std::move(r)), query(std::move(q)) {}
  Eigen::Vector2d ref = Eigen::Vector2d::Zero();
  Eigen::Vector2d query = Eigen::Vector2d::Zero();
};

std::pair<Eigen::Isometry2d, std::size_t> ransacAlignment2D(
    const std::vector<PointPair>& keypoint_pairs, double inlier_threshold);

}  // namespace map_closures
