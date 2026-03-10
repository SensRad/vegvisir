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
#include "pose_graph_optimizer.hpp"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/isometry3d_gradients.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <iostream>
#include <memory>

namespace {
constexpr double epsilon = 1e-9; // Allow more iterations before early stop

// Binary edge for GNSS position constraints WITH alignment estimation.
// Vertex 0: Pose in map frame
// Vertex 1: T_ENU_map alignment transform
// Measurement: GNSS position in ENU frame
// Error: T_ENU_map * p_map - p_ENU_gnss
class EdgeGnssWithAlignment
    : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSE3,
                                 g2o::VertexSE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeGnssWithAlignment() = default;

  void computeError() override {
    const g2o::VertexSE3 *v_pose =
        static_cast<const g2o::VertexSE3 *>(_vertices[0]);
    const g2o::VertexSE3 *v_align =
        static_cast<const g2o::VertexSE3 *>(_vertices[1]);

    // Position in map frame
    Eigen::Vector3d p_map = v_pose->estimate().translation();

    // Transform to ENU frame: p_ENU = T_ENU_map * p_map
    Eigen::Vector3d p_enu_estimated = v_align->estimate() * p_map;

    // Error = estimated ENU position - measured GNSS ENU position
    _error = p_enu_estimated - _measurement;
  }

  bool read(std::istream &is) override {
    is >> _measurement[0] >> _measurement[1] >> _measurement[2];
    for (int i = 0; i < 3; ++i) {
      for (int j = i; j < 3; ++j) {
        is >> information()(i, j);
        if (i != j) {
          information()(j, i) = information()(i, j);
        }
      }
    }
    return true;
  }

  bool write(std::ostream &os) const override {
    os << _measurement[0] << " " << _measurement[1] << " " << _measurement[2];
    for (int i = 0; i < 3; ++i) {
      for (int j = i; j < 3; ++j) {
        os << " " << information()(i, j);
      }
    }
    return os.good();
  }
};

// Binary edge for full SE3 GNSS pose constraints WITH alignment estimation.
// Vertex 0: Pose in map frame
// Vertex 1: T_ENU_map alignment transform
// Measurement: Full SE3 pose in ENU frame (from INS)
// Error: log(T_gnss_enu^{-1} * T_ENU_map * T_map_pose) — 6D
class EdgeGnssPoseWithAlignment
    : public g2o::BaseBinaryEdge<6, g2o::Isometry3, g2o::VertexSE3,
                                 g2o::VertexSE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeGnssPoseWithAlignment() = default;

  void computeError() override {
    const auto *v_pose = static_cast<const g2o::VertexSE3 *>(_vertices[0]);
    const auto *v_align = static_cast<const g2o::VertexSE3 *>(_vertices[1]);

    // T_enu_pose_estimated = T_ENU_map * T_map_pose
    g2o::Isometry3 estimated = v_align->estimate() * v_pose->estimate();

    // Error = log(T_gnss_enu^{-1} * T_enu_pose_estimated)
    // Using g2o's toVectorMQT: [tx, ty, tz, qx, qy, qz] (minimal quaternion)
    g2o::Isometry3 delta = _inverseMeasurement * estimated;
    _error = g2o::internal::toVectorMQT(delta);
  }

  void setMeasurement(const g2o::Isometry3 &m) override {
    _measurement = m;
    _inverseMeasurement = m.inverse();
  }

  bool read(std::istream &is) override {
    Eigen::Matrix<double, 7, 1> v;
    for (int i = 0; i < 7; ++i) {
      is >> v[i];
    }
    _measurement = g2o::internal::fromVectorQT(v);
    _inverseMeasurement = _measurement.inverse();
    for (int i = 0; i < 6; ++i) {
      for (int j = i; j < 6; ++j) {
        is >> information()(i, j);
        if (i != j) {
          information()(j, i) = information()(i, j);
        }
      }
    }
    return true;
  }

  bool write(std::ostream &os) const override {
    Eigen::Matrix<double, 7, 1> v = g2o::internal::toVectorQT(_measurement);
    for (int i = 0; i < 7; ++i) {
      os << v[i] << " ";
    }
    for (int i = 0; i < 6; ++i) {
      for (int j = i; j < 6; ++j) {
        os << " " << information()(i, j);
      }
    }
    return os.good();
  }

private:
  g2o::Isometry3 _inverseMeasurement;
};

} // namespace

// clang-format off
namespace g2o {
G2O_REGISTER_TYPE(VERTEX_SE3:QUAT, VertexSE3)
G2O_REGISTER_TYPE(EDGE_SE3:QUAT, EdgeSE3)
G2O_REGISTER_TYPE(EDGE_GNSS_WITH_ALIGNMENT, EdgeGnssWithAlignment)
G2O_REGISTER_TYPE(EDGE_GNSS_POSE_WITH_ALIGNMENT, EdgeGnssPoseWithAlignment)
}  // namespace g2o
// clang-format on

namespace pgo {
using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
using LinearSolverType =
    g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
using AlgorithmType = g2o::OptimizationAlgorithmDogleg;

PoseGraphOptimizer::PoseGraphOptimizer(const int max_iterations, bool verbose)
    : max_iterations_(max_iterations) {
  graph = std::make_unique<g2o::SparseOptimizer>();
  graph->setVerbose(verbose);

  auto solver = new AlgorithmType(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

  auto terminateAction = new g2o::SparseOptimizerTerminateAction;
  terminateAction->setGainThreshold(epsilon);
  graph->addPostIterationAction(terminateAction);
  graph->setAlgorithm(solver);
}

void PoseGraphOptimizer::fixVariable(const int id) {
  auto *vertex = graph->vertex(id);
  if (vertex) {
    vertex->setFixed(true);
  }
}

void PoseGraphOptimizer::addVariable(const int id, const Eigen::Matrix4d &T) {
  Eigen::Isometry3d pose;
  pose.matrix() = T;
  g2o::VertexSE3 *variable = new g2o::VertexSE3();
  variable->setId(id);
  variable->setEstimate(pose);
  graph->addVertex(variable);
}

void PoseGraphOptimizer::addFactor(const int id_source, const int id_target,
                                   const Eigen::Matrix4d &T,
                                   const Eigen::Matrix6d &information_matrix) {
  Eigen::Isometry3d relative_pose;
  relative_pose.matrix() = T;
  g2o::EdgeSE3 *factor = new g2o::EdgeSE3();
  factor->setVertex(0, graph->vertex(id_target));
  factor->setVertex(1, graph->vertex(id_source));
  factor->setInformation(information_matrix);
  factor->setMeasurement(relative_pose);
  graph->addEdge(factor);
}

void PoseGraphOptimizer::initializeAlignmentVariable(
    const Eigen::Matrix4d &initial_estimate) {
  Eigen::Isometry3d T_enu_map;
  T_enu_map.matrix() = initial_estimate;

  g2o::VertexSE3 *alignment_vertex = new g2o::VertexSE3();
  alignment_vertex->setId(ALIGNMENT_VERTEX_ID);
  alignment_vertex->setEstimate(T_enu_map);
  alignment_vertex->setFixed(false);
  graph->addVertex(alignment_vertex);
  alignment_initialized_ = true;
}

void PoseGraphOptimizer::addGnssConstraintWithAlignment(
    const int pose_id, const Eigen::Vector3d &position_enu,
    const Eigen::Matrix3d &information_matrix, double huber_delta) {

  if (!alignment_initialized_) {
    std::cerr << "Error: Alignment variable not initialized. Call "
                 "initializeAlignmentVariable() first."
              << std::endl;
    return;
  }

  // Verify both vertices exist before creating edge
  auto *pose_vertex = graph->vertex(pose_id);
  auto *align_vertex = graph->vertex(ALIGNMENT_VERTEX_ID);

  if (!pose_vertex) {
    std::cerr << "Error: Pose vertex " << pose_id << " not found in graph. "
              << "Skipping GNSS constraint." << std::endl;
    return;
  }

  if (!align_vertex) {
    std::cerr << "Error: Alignment vertex (ID=" << ALIGNMENT_VERTEX_ID
              << ") not found in graph. This may indicate a g2o vertex lookup "
              << "issue. Skipping GNSS constraint." << std::endl;
    return;
  }

  EdgeGnssWithAlignment *gnss_edge = new EdgeGnssWithAlignment();
  gnss_edge->setVertex(0, pose_vertex);
  gnss_edge->setVertex(1, align_vertex);
  gnss_edge->setMeasurement(position_enu);
  gnss_edge->setInformation(information_matrix);

  // Apply Huber robust kernel if requested
  if (huber_delta > 0) {
    g2o::RobustKernelHuber *huber = new g2o::RobustKernelHuber();
    huber->setDelta(huber_delta);
    gnss_edge->setRobustKernel(huber);
  }

  graph->addEdge(gnss_edge);
}

void PoseGraphOptimizer::addGnssPoseConstraintWithAlignment(
    const int pose_id, const Eigen::Matrix4d &pose_enu,
    const Eigen::Matrix6d &information_matrix, double huber_delta) {

  if (!alignment_initialized_) {
    std::cerr << "Error: Alignment variable not initialized. Call "
                 "initializeAlignmentVariable() first."
              << std::endl;
    return;
  }

  auto *pose_vertex = graph->vertex(pose_id);
  auto *align_vertex = graph->vertex(ALIGNMENT_VERTEX_ID);

  if (!pose_vertex) {
    std::cerr << "Error: Pose vertex " << pose_id << " not found in graph. "
              << "Skipping GNSS pose constraint." << std::endl;
    return;
  }

  if (!align_vertex) {
    std::cerr << "Error: Alignment vertex (ID=" << ALIGNMENT_VERTEX_ID
              << ") not found in graph. Skipping GNSS pose constraint."
              << std::endl;
    return;
  }

  Eigen::Isometry3d measurement;
  measurement.matrix() = pose_enu;

  EdgeGnssPoseWithAlignment *edge = new EdgeGnssPoseWithAlignment();
  edge->setVertex(0, pose_vertex);
  edge->setVertex(1, align_vertex);
  edge->setMeasurement(measurement);
  edge->setInformation(information_matrix);

  if (huber_delta > 0) {
    g2o::RobustKernelHuber *huber = new g2o::RobustKernelHuber();
    huber->setDelta(huber_delta);
    edge->setRobustKernel(huber);
  }

  graph->addEdge(edge);
}

Eigen::Matrix4d PoseGraphOptimizer::getAlignmentTransform() const {
  if (!alignment_initialized_) {
    return Eigen::Matrix4d::Identity();
  }
  g2o::VertexSE3 *v =
      static_cast<g2o::VertexSE3 *>(graph->vertex(ALIGNMENT_VERTEX_ID));
  if (!v) {
    return Eigen::Matrix4d::Identity();
  }
  return v->estimate().matrix();
}

bool PoseGraphOptimizer::hasAlignmentVariable() const {
  return alignment_initialized_;
}

PoseGraphOptimizer::PoseIDMap PoseGraphOptimizer::estimates() const {
  const g2o::HyperGraph::VertexIDMap &variables = graph->vertices();
  PoseIDMap poses;
  for (const auto &[id, v] : variables) {
    // Skip the alignment vertex
    if (id == ALIGNMENT_VERTEX_ID) {
      continue;
    }
    Eigen::Isometry3d pose = static_cast<g2o::VertexSE3 *>(v)->estimate();
    poses[id] = pose.matrix();
  }
  return poses;
}

void PoseGraphOptimizer::optimize() {
  graph->initializeOptimization();
  graph->optimize(max_iterations_);
}
} // namespace pgo
