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

#include "AlignRansac2D.hpp"

#include <cmath>

#include <algorithm>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace {

// Compute rigid transform (rotation + translation) using Kabsch-Umeyama
Eigen::Isometry2d kabschUmeyamaAlignment2D(
    const std::vector<map_closures::PointPair>& keypoint_pairs) {
  const auto n = static_cast<double>(keypoint_pairs.size());
  Eigen::Vector2d mean_ref = Eigen::Vector2d::Zero();
  Eigen::Vector2d mean_query = Eigen::Vector2d::Zero();
  for (const auto& kp : keypoint_pairs) {
    mean_ref += kp.ref;
    mean_query += kp.query;
  }
  mean_ref /= n;
  mean_query /= n;

  Eigen::Matrix2d covariance_matrix = Eigen::Matrix2d::Zero();
  for (const auto& kp : keypoint_pairs) {
    covariance_matrix += (kp.ref - mean_ref) * (kp.query - mean_query).transpose();
  }

  const Eigen::JacobiSVD<Eigen::Matrix2d> svd(covariance_matrix,
                                              Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Standard Kabsch reflection correction
  Eigen::Matrix2d reflection_correction = Eigen::Matrix2d::Identity();
  reflection_correction(1, 1) = (svd.matrixV() * svd.matrixU().transpose()).determinant();

  Eigen::Isometry2d alignment = Eigen::Isometry2d::Identity();
  alignment.linear() = svd.matrixV() * reflection_correction * svd.matrixU().transpose();
  alignment.translation() = mean_query - alignment.linear() * mean_ref;

  return alignment;
}

}  // namespace

namespace map_closures {

std::pair<Eigen::Isometry2d, std::size_t> ransacAlignment2D(
    const std::vector<PointPair>& keypoint_pairs) {
  if (keypoint_pairs.size() < 2) {
    return {Eigen::Isometry2d::Identity(), 0};
  }

  const auto n = static_cast<int>(keypoint_pairs.size());

  std::vector<PointPair> sample_keypoint_pairs(2);
  std::vector<int> inlier_indices;
  inlier_indices.reserve(n);

  std::vector<int> optimal_inlier_indices;
  optimal_inlier_indices.reserve(n);

  std::mt19937 rng{std::random_device{}()};

  auto find_inliers = [&](const Eigen::Isometry2d& transform, std::vector<int>& out) {
    out.clear();
    for (int i = 0; i < n; ++i) {
      if ((transform * keypoint_pairs[i].ref - keypoint_pairs[i].query).norm() <
          RANSAC_INLIER_THRESHOLD) {
        out.emplace_back(i);
      }
    }
  };

  auto fit_on_indices = [&](const std::vector<int>& indices) {
    std::vector<PointPair> subset;
    subset.reserve(indices.size());
    for (const auto idx : indices) {
      subset.push_back(keypoint_pairs[idx]);
    }
    return kabschUmeyamaAlignment2D(subset);
  };

  int max_iterations = RANSAC_MAX_TRIALS;
  for (int iter = 0; iter < max_iterations; ++iter) {
    std::sample(keypoint_pairs.begin(), keypoint_pairs.end(), sample_keypoint_pairs.begin(), 2,
                rng);
    const auto transform = kabschUmeyamaAlignment2D(sample_keypoint_pairs);
    find_inliers(transform, inlier_indices);

    if (inlier_indices.size() <= optimal_inlier_indices.size()) {
      continue;
    }

    optimal_inlier_indices = inlier_indices;

    // Adaptive termination: reduce max_iterations based on observed inlier
    // ratio
    const double w = static_cast<double>(optimal_inlier_indices.size()) / n;
    const double denom = std::log(1.0 - std::pow(w, RANSAC_MIN_POINTS));
    if (std::abs(denom) < 1e-12) {
      break;
    }
    max_iterations = std::min(
        max_iterations,
        std::max(RANSAC_MIN_TRIALS,
                 static_cast<int>(std::ceil(std::log(1.0 - RANSAC_PROBABILITY_SUCCESS) / denom))));
  }

  if (optimal_inlier_indices.size() < 2) {
    return {Eigen::Isometry2d::Identity(), 0};
  }

  // Final refit on all inliers
  const auto transform = fit_on_indices(optimal_inlier_indices);
  return {transform, optimal_inlier_indices.size()};
}

}  // namespace map_closures
