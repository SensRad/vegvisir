// Copyright (c) Sensrad 2025-2026

#include "VegvisirPGO.hpp"

#include <unordered_map>

#include "pgo/pose_graph_optimizer.hpp"

namespace vegvisir {

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
FineGrainedPGOResult runFineGrainedPGO(const LocalMapGraph& local_map_graph,
                                       const std::vector<map_closures::ClosureCandidate>& closures,
                                       const GnssState& gnss_state, const VegvisirConfig& config) {
  FineGrainedPGOResult result;

  // Conditional parameters based on GNSS availability
  const bool has_gnss = gnss_state.hasGnss();
  const int max_iterations =
      has_gnss ? (config.pgo_max_iterations * 10) : config.pgo_max_iterations;
  const double odom_weight = 0.01;
  const double closure_weight = 1.0;  // Loop closures are strong constraints

  pgo::PoseGraphOptimizer pgo(max_iterations);

  // Initialize alignment variable if we have GNSS
  if (has_gnss) {
    if (gnss_state.has_initial_alignment) {
      pgo.initializeAlignmentVariable(gnss_state.initial_pose_enu_map);
    } else {
      pgo.initializeAlignmentVariable(Eigen::Matrix4d::Identity());
    }
  }

  int id = 0;
  bool first = true;
  Eigen::Matrix4d prev_last_world_pose = Eigen::Matrix4d::Identity();

  // Build information matrix with odometry weight
  const Eigen::Matrix<double, 6, 6> odom_info =
      odom_weight * Eigen::Matrix<double, 6, 6>::Identity();

  // Build information matrix for loop closures
  const Eigen::Matrix<double, 6, 6> closure_info =
      closure_weight * Eigen::Matrix<double, 6, 6>::Identity();

  // Map from keypose (node) ID to first per-frame pose ID
  std::unordered_map<int, int> keypose_to_pose_id;

  // Map from sequential odom frame index to PGO vertex ID.
  // Non-first nodes have traj[0]=Identity (keypose anchor) which is a PGO
  // vertex but NOT an odom frame, so PGO vertex IDs diverge from odom indices.
  std::vector<int> odom_to_pgo;

  for (const auto& [node_key, node] : local_map_graph) {
    const auto& traj = node.localTrajectory();

    if (first) {
      const Eigen::Matrix4d first_pose = traj.empty() ? node.keypose() : node.keypose() * traj[0];
      pgo.addVariable(id, first_pose);
      pgo.fixVariable(id);
      odom_to_pgo.push_back(id);  // Node 0: traj[0] IS an odom frame
      first = false;
    } else {
      // Add the first trajectory pose of this node as a new vertex
      // with an inter-node odometry factor from previous node's last pose.
      // traj[0]=Identity is a keypose anchor, not an odom frame.
      const Eigen::Matrix4d first_world = traj.empty() ? node.keypose() : node.keypose() * traj[0];
      ++id;
      pgo.addVariable(id, first_world);
      const Eigen::Matrix4d inter_node_factor = prev_last_world_pose.inverse() * first_world;
      pgo.addFactor(id, id - 1, inter_node_factor, odom_info);
      // Don't push to odom_to_pgo — this vertex has no corresponding odom frame
    }

    // Record mapping from keypose ID to first pose ID (after adding traj[0])
    keypose_to_pose_id[static_cast<int>(node_key)] = id;

    // Build odometry factors between consecutive trajectory poses
    for (size_t i = 0; i + 1 < traj.size(); ++i) {
      const Eigen::Matrix4d factor = traj[i].inverse() * traj[i + 1];
      pgo.addVariable(id + 1, node.keypose() * traj[i + 1]);
      pgo.addFactor(id + 1, id, factor, odom_info);
      ++id;
      odom_to_pgo.push_back(id);  // traj[i+1] is always a real odom frame
    }

    // Track last world-frame pose for inter-node factor computation
    prev_last_world_pose = traj.empty() ? node.keypose() : node.keypose() * traj.back();
  }

  // Fix the last vertex when there's no GNSS to anchor the graph
  if (!has_gnss && id > 0) {
    pgo.fixVariable(id);
  }

  // Add loop closure constraints
  for (const auto& closure : closures) {
    auto src_it = keypose_to_pose_id.find(closure.source_id);
    auto tgt_it = keypose_to_pose_id.find(closure.target_id);

    if (src_it != keypose_to_pose_id.end() && tgt_it != keypose_to_pose_id.end()) {
      // closure.pose transforms source-local points to target-local frame:
      // T * p_source_local ≈ p_target_local, i.e. closure.pose ≈ keypose_tgt⁻¹
      // * keypose_src Same convention as the online keypose optimizer in
      // SlamBackend::applyAcceptedClosure
      pgo.addFactor(src_it->second, tgt_it->second, closure.pose, closure_info);
    }
  }

  // Add GNSS position constraints with alignment estimation.
  // pose_index is an odom frame index — translate to PGO vertex ID.
  for (const auto& gnss : gnss_state.measurements()) {
    if (gnss.pose_index >= 0 && gnss.pose_index < static_cast<int>(odom_to_pgo.size())) {
      const int pgo_id = odom_to_pgo[gnss.pose_index];
      pgo.addGnssConstraintWithAlignment(pgo_id, gnss.position_enu, gnss.information_matrix);
    }
  }

  // Add full SE3 GNSS pose constraints with alignment estimation.
  // pose_index is an odom frame index — translate to PGO vertex ID.
  for (const auto& gnss_pose : gnss_state.poseMeasurements()) {
    if (gnss_pose.pose_index >= 0 && gnss_pose.pose_index < static_cast<int>(odom_to_pgo.size())) {
      const int pgo_id = odom_to_pgo[gnss_pose.pose_index];
      pgo.addGnssPoseConstraintWithAlignment(pgo_id, gnss_pose.pose_enu,
                                             gnss_pose.information_matrix);
    }
  }

  pgo.optimize();

  result.alignment_transform = has_gnss ? pgo.getAlignmentTransform() : Eigen::Matrix4d::Identity();

  // Collect optimised poses in insertion order
  const auto& estimates = pgo.estimates();
  result.optimized_poses.reserve(estimates.size());
  for (const auto& [_, pose] : estimates) {
    result.optimized_poses.push_back(pose);
  }

  // Recover optimized keyposes from PGO vertices
  for (const auto& [node_key, node] : local_map_graph) {
    const int node_id = static_cast<int>(node_key);
    auto it = keypose_to_pose_id.find(node_id);
    if (it == keypose_to_pose_id.end()) {
      continue;
    }
    const int pose_id = it->second;
    const auto& traj = node.localTrajectory();
    const auto est_it = estimates.find(pose_id);
    if (est_it == estimates.end()) {
      continue;
    }
    result.optimized_keyposes[node_id] =
        traj.empty() ? est_it->second : est_it->second * traj[0].inverse();
  }

  return result;
}

}  // namespace vegvisir
