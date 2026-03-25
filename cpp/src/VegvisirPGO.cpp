// Copyright (c) Sensrad 2026

#include "VegvisirPGO.hpp"

#include <unordered_map>

#include "pgo/pose_graph_optimizer.hpp"

namespace vegvisir {
namespace {

struct PoseGraphMappings {
  std::unordered_map<int, int> keypose_to_pose_id;
  std::vector<int> odom_to_pgo;
  int last_vertex_id = 0;
};

Eigen::Matrix<double, 6, 6> makeInformationMatrix(double weight) {
  return weight * Eigen::Matrix<double, 6, 6>::Identity();
}

void initializeAlignmentFromGnss(pgo::PoseGraphOptimizer& optimizer, const GnssState& gnss_state) {
  if (!gnss_state.hasGnss())
    return;

  const Eigen::Matrix4d initial_alignment = gnss_state.has_initial_alignment
                                                ? gnss_state.initial_pose_enu_map
                                                : Eigen::Matrix4d::Identity();
  optimizer.initializeAlignmentVariable(initial_alignment);
}

Eigen::Matrix4d computeFirstWorldPose(const LocalMap& node) {
  const auto& traj = node.localTrajectory();
  return traj.empty() ? node.keypose() : node.keypose() * traj[0];
}

Eigen::Matrix4d computeLastWorldPose(const LocalMap& node) {
  const auto& traj = node.localTrajectory();
  return traj.empty() ? node.keypose() : node.keypose() * traj.back();
}

void addFirstNode(pgo::PoseGraphOptimizer& optimizer, const LocalMap& node,
                  PoseGraphMappings& mappings) {
  const Eigen::Matrix4d first_pose = computeFirstWorldPose(node);
  optimizer.addVariable(mappings.last_vertex_id, first_pose);
  optimizer.fixVariable(mappings.last_vertex_id);
  mappings.odom_to_pgo.push_back(mappings.last_vertex_id);
}

void addSubsequentNode(pgo::PoseGraphOptimizer& optimizer, const LocalMap& node,
                       const Eigen::Matrix4d& prev_last_world_pose,
                       const Eigen::Matrix<double, 6, 6>& odom_info, PoseGraphMappings& mappings) {
  const Eigen::Matrix4d first_world = computeFirstWorldPose(node);
  ++mappings.last_vertex_id;
  optimizer.addVariable(mappings.last_vertex_id, first_world);

  const Eigen::Matrix4d inter_node_factor = prev_last_world_pose.inverse() * first_world;
  optimizer.addFactor(mappings.last_vertex_id, mappings.last_vertex_id - 1, inter_node_factor,
                      odom_info);
}

void addTrajectoryFactors(pgo::PoseGraphOptimizer& optimizer, const LocalMap& node,
                          const Eigen::Matrix<double, 6, 6>& odom_info,
                          PoseGraphMappings& mappings) {
  const auto& traj = node.localTrajectory();
  for (size_t i = 0; i + 1 < traj.size(); ++i) {
    const Eigen::Matrix4d factor = traj[i].inverse() * traj[i + 1];
    optimizer.addVariable(mappings.last_vertex_id + 1, node.keypose() * traj[i + 1]);
    optimizer.addFactor(mappings.last_vertex_id + 1, mappings.last_vertex_id, factor, odom_info);
    ++mappings.last_vertex_id;
    mappings.odom_to_pgo.push_back(mappings.last_vertex_id);
  }
}

PoseGraphMappings buildOdometryGraph(pgo::PoseGraphOptimizer& optimizer,
                                     const LocalMapGraph& local_map_graph,
                                     const Eigen::Matrix<double, 6, 6>& odom_info) {
  PoseGraphMappings mappings;
  bool first = true;
  Eigen::Matrix4d prev_last_world_pose = Eigen::Matrix4d::Identity();

  for (const auto& [node_key, node] : local_map_graph) {
    if (first) {
      addFirstNode(optimizer, node, mappings);
      first = false;
    } else {
      addSubsequentNode(optimizer, node, prev_last_world_pose, odom_info, mappings);
    }

    mappings.keypose_to_pose_id[static_cast<int>(node_key)] = mappings.last_vertex_id;
    addTrajectoryFactors(optimizer, node, odom_info, mappings);
    prev_last_world_pose = computeLastWorldPose(node);
  }

  return mappings;
}

void addClosureFactors(pgo::PoseGraphOptimizer& optimizer,
                       const std::vector<map_closures::ClosureCandidate>& closures,
                       const std::unordered_map<int, int>& keypose_to_pose_id,
                       const Eigen::Matrix<double, 6, 6>& closure_info) {
  for (const auto& closure : closures) {
    const auto src_it = keypose_to_pose_id.find(closure.source_id);
    const auto tgt_it = keypose_to_pose_id.find(closure.target_id);

    if (src_it != keypose_to_pose_id.end() && tgt_it != keypose_to_pose_id.end()) {
      // closure.pose ≈ keypose_tgt⁻¹ * keypose_src (same convention as
      // SlamBackend::applyAcceptedClosure)
      optimizer.addFactor(src_it->second, tgt_it->second, closure.pose, closure_info);
    }
  }
}

void addGnssFactors(pgo::PoseGraphOptimizer& optimizer, const GnssState& gnss_state,
                    const std::vector<int>& odom_to_pgo) {
  const auto odom_size = static_cast<int>(odom_to_pgo.size());

  for (const auto& gnss : gnss_state.measurements()) {
    if (gnss.pose_index >= 0 && gnss.pose_index < odom_size) {
      optimizer.addGnssConstraintWithAlignment(odom_to_pgo[gnss.pose_index], gnss.position_enu,
                                               gnss.information_matrix);
    }
  }

  for (const auto& gnss_pose : gnss_state.poseMeasurements()) {
    if (gnss_pose.pose_index >= 0 && gnss_pose.pose_index < odom_size) {
      optimizer.addGnssPoseConstraintWithAlignment(
          odom_to_pgo[gnss_pose.pose_index], gnss_pose.pose_enu, gnss_pose.information_matrix);
    }
  }
}

FineGrainedPGOResult summarizeResults(const pgo::PoseGraphOptimizer& optimizer,
                                      const LocalMapGraph& local_map_graph,
                                      const std::unordered_map<int, int>& keypose_to_pose_id,
                                      bool has_gnss) {
  FineGrainedPGOResult result;
  result.alignment_transform =
      has_gnss ? optimizer.getAlignmentTransform() : Eigen::Matrix4d::Identity();

  const auto& estimates = optimizer.estimates();
  result.optimized_poses.reserve(estimates.size());
  for (const auto& [_, pose] : estimates) {
    result.optimized_poses.push_back(pose);
  }

  for (const auto& [node_key, node] : local_map_graph) {
    const int node_id = static_cast<int>(node_key);
    const auto it = keypose_to_pose_id.find(node_id);
    if (it == keypose_to_pose_id.end())
      continue;

    const auto est_it = estimates.find(it->second);
    if (est_it == estimates.end())
      continue;

    const auto& traj = node.localTrajectory();
    result.optimized_keyposes[node_id] =
        traj.empty() ? est_it->second : est_it->second * traj[0].inverse();
  }

  return result;
}

}  // namespace

FineGrainedPGOResult runFineGrainedPGO(const LocalMapGraph& local_map_graph,
                                       const std::vector<map_closures::ClosureCandidate>& closures,
                                       const GnssState& gnss_state, const VegvisirConfig& config) {
  const bool has_gnss = gnss_state.hasGnss();
  const int max_iterations =
      has_gnss ? (config.pgo_max_iterations * 10) : config.pgo_max_iterations;

  constexpr double kOdometryWeight = 0.01;
  constexpr double kClosureWeight = 1.0;
  const auto odom_info = makeInformationMatrix(kOdometryWeight);
  const auto closure_info = makeInformationMatrix(kClosureWeight);

  pgo::PoseGraphOptimizer optimizer(max_iterations);
  initializeAlignmentFromGnss(optimizer, gnss_state);

  auto mappings = buildOdometryGraph(optimizer, local_map_graph, odom_info);

  if (!has_gnss && mappings.last_vertex_id > 0) {
    optimizer.fixVariable(mappings.last_vertex_id);
  }

  addClosureFactors(optimizer, closures, mappings.keypose_to_pose_id, closure_info);
  addGnssFactors(optimizer, gnss_state, mappings.odom_to_pgo);

  optimizer.optimize();

  return summarizeResults(optimizer, local_map_graph, mappings.keypose_to_pose_id, has_gnss);
}

}  // namespace vegvisir
