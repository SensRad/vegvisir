// Copyright (c) Sensrad 2025-2026
//
// MIT License
//
// Copyright (c) 2024 Saurabh Gupta, Tiziano Guadagnino, Benedikt Mersch,
// Ignacio Vizzo, Cyrill Stachniss.
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

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>
#include <vector>

namespace map_closures {

static constexpr double RANSAC_INLIER_THRESHOLD = 4.0;
static constexpr double RANSAC_INLIERS_RATIO = 0.3;
static constexpr double RANSAC_PROBABILITY_SUCCESS = 0.999;
static constexpr int RANSAC_MIN_POINTS = 2;
static constexpr int RANSAC_MIN_TRIALS = 100;
static constexpr int RANSAC_MAX_TRIALS = 100;

struct PointPair {
  PointPair() = default;
  PointPair(const Eigen::Vector2d &r, const Eigen::Vector2d &q)
      : ref(r), query(q) {}
  Eigen::Vector2d ref = Eigen::Vector2d::Zero();
  Eigen::Vector2d query = Eigen::Vector2d::Zero();
};

std::pair<Eigen::Isometry2d, std::size_t>
RansacAlignment2D(const std::vector<PointPair> &keypoint_pairs);

} // namespace map_closures
