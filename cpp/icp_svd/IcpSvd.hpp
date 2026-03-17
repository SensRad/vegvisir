// Copyright (c) Sensrad 2026
// Lightweight SVD-based point-to-point ICP

#pragma once

#include <cmath>
#include <cstdint>

#include <algorithm>
#include <limits>
#include <numeric>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "voxel_map/VoxelMap.hpp"

namespace icp {

struct Result {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  bool converged = false;
  int iterations = 0;
};

class IcpSvd {
 public:
  // Align source_points onto target_points using point-to-point ICP.
  //
  // Builds a VoxelMap from target_points for nearest-neighbor queries,
  // then iterates SVD-based rigid alignment until convergence or
  // max_iterations is reached.
  static Result pointToPointICP(const std::vector<Eigen::Vector3d>& source_points,
                                const std::vector<Eigen::Vector3d>& target_points,
                                const Eigen::Matrix4d& initial_guess, double voxel_size,
                                int max_iterations, double convergence_threshold,
                                double max_correspondence_distance);

 private:
  static constexpr double TRIM_RATIO = 0.75;
  static constexpr double ROT_CONVERGENCE_SCALE = 2.0;
  static constexpr size_t MIN_CORRESPONDENCES = 6;

  // Build a 4x4 SE3 matrix from a 3x3 rotation and 3x1 translation.
  static Eigen::Matrix4d makeSE3(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

  // Extract the rotation angle (radians) from a 3x3 rotation matrix
  // using the trace formula: angle = acos((trace(R) - 1) / 2).
  static double rotationAngle(const Eigen::Matrix3d& R);

  // Trim correspondences to the closest TRIM_RATIO fraction.
  // Reorders indices[0..num_corr-1] so that the closest `keep` entries come
  // first. Returns the number of surviving correspondences.
  static size_t trimCorrespondences(uint32_t *indices, const double *dist_sq, size_t num_corr);

  // Compute Cauchy-weighted centroids and cross-covariance from
  // correspondences, then solve the rigid alignment via SVD.
  // Returns {dR, dt} or nullopt if the SVD is degenerate.
  static std::optional<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> computeAlignment(
      const Eigen::Vector3d *src, const Eigen::Vector3d *tgt, const double *dist_sq,
      const uint32_t *indices, size_t count, double cauchy_c_sq);
};

}  // namespace icp
