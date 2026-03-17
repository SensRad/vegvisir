// Copyright (c) Sensrad 2026

// MIT License

// Copyright (c) 2025 Tiziano Guadagnino

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once
#include <g2o/core/sparse_optimizer.h>

#include <Eigen/Geometry>
#include <fstream>
#include <memory>
#include <string>

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
} // namespace Eigen

namespace pgo {

// Special vertex ID for the ENU-to-map alignment transform.
// Uses a large positive ID to avoid collision with pose vertex IDs.
// Note: Negative IDs can cause issues with some g2o versions where
// vertex lookup fails for negative keys.
static constexpr int ALIGNMENT_VERTEX_ID = 1000000;

class PoseGraphOptimizer {
public:
  using PoseIDMap = std::map<int, Eigen::Matrix4d>;
  explicit PoseGraphOptimizer(int max_iterations, bool verbose = false);

  void fixVariable(int id);
  void addVariable(int id, const Eigen::Matrix4d &t);

  void addFactor(int id_source, int id_target,
                 const Eigen::Matrix4d &t,
                 const Eigen::Matrix6d &information_matrix);

  // Initialize the ENU-to-map alignment transform variable.
  // This should be called once before adding GNSS constraints.
  // @param initial_estimate Initial guess for T_ENU_map
  void initializeAlignmentVariable(const Eigen::Matrix4d &initial_estimate);

  // Add a GNSS position constraint with alignment estimation.
  // The GNSS position should be in ENU frame. The optimization will estimate
  // T_ENU_map such that: p_ENU = T_ENU_map * p_map
  // @param pose_id The vertex ID of the pose in map frame
  // @param position_enu The GNSS XYZ position in ENU frame
  // @param information_matrix 3x3 information matrix (inverse covariance)
  // @param huber_delta Huber loss delta for robustness (0 = no robust loss)
  void addGnssConstraintWithAlignment(int pose_id,
                                      const Eigen::Vector3d &position_enu,
                                      const Eigen::Matrix3d &information_matrix,
                                      double huber_delta = 0.0);

  // Add a full SE3 GNSS pose constraint with alignment estimation.
  // The GNSS pose should be in ENU frame. The optimization will estimate
  // T_ENU_map such that: T_ENU_pose = T_ENU_map * T_map_pose
  // @param pose_id The vertex ID of the pose in map frame
  // @param pose_enu The full SE3 pose in ENU frame (4x4 matrix)
  // @param information_matrix 6x6 information matrix (inverse covariance)
  // @param huber_delta Huber loss delta for robustness (0 = no robust loss)
  void addGnssPoseConstraintWithAlignment(
      int pose_id, const Eigen::Matrix4d &pose_enu,
      const Eigen::Matrix6d &information_matrix, double huber_delta = 0.0);

  // Get the optimized alignment transform T_ENU_map after optimization.
  [[nodiscard]] Eigen::Matrix4d getAlignmentTransform() const;

  // Check if alignment variable has been initialized.
  [[nodiscard]] bool hasAlignmentVariable() const;

  [[nodiscard]] PoseIDMap estimates() const;

  void readGraph(const std::string &filename) {
    std::ifstream file(filename.c_str());
    graph_->clear();
    graph_->load(file);
  }
  void writeGraph(const std::string &filename) const {
    graph_->save(filename.c_str());
  }

  void optimize();

private:
  std::unique_ptr<g2o::SparseOptimizer> graph_;
  int max_iterations_;
  bool alignment_initialized_ = false;
};
} // namespace pgo
