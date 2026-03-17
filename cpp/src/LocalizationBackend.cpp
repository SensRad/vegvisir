// Copyright (c) Sensrad 2025-2026

#include "LocalizationBackend.hpp"
#include "Vegvisir.hpp"

#include <iostream>

namespace vegvisir {

LocalizationBackend::LocalizationBackend(Vegvisir &vegvisir)
    : VegvisirBackend(vegvisir) {}

void LocalizationBackend::initialize() {
  localization_anchor_initialized_ = false;
  pose_odom_anchor_.setIdentity();
}

void LocalizationBackend::preIntegrate(const Eigen::Matrix4d &pose_odom_base,
                                       const Sophus::SE3d &delta_pose) {
  // Localization: ring-buffer submaps
  initLocalizationAnchor(pose_odom_base);

  // current_pose_ is anchor <- base
  vegvisir_.current_pose_ = pose_odom_anchor_.inverse() * pose_odom_base;

  // Lock: pose_filter_ and tf_map_odom_ are written by background closure thread
  std::lock_guard<std::mutex> lock(vegvisir_.closure_mutex_);
  vegvisir_.pose_filter_.predict(delta_pose);
  vegvisir_.tf_map_odom_ = vegvisir_.pose_filter_.state().matrix();
}

void LocalizationBackend::postIntegrate() {
  // Don't grow trajectory unbounded in localization; store only the latest
  vegvisir_.local_map_graph_.lastLocalMap().localTrajectory().assign(
      1, vegvisir_.current_pose_);
}

double LocalizationBackend::queryDistanceM() const {
  return Vegvisir::QUERY_DISTANCE_LOCALIZATION_M;
}

void LocalizationBackend::runQueryCycle(const Eigen::Matrix4d &pose_odom_base) {
  // Localization: build query cloud from ring buffer, then query closures
  std::vector<Eigen::Vector3d> query_points_mc;
  std::vector<Eigen::Vector3d> query_points_icp;

  buildLocalizationQueryCloudInBaseFrame(pose_odom_base, query_points_mc,
                                         query_points_icp);

  // Cut submap before async closure call (independent, must stay synchronous)
  cutLocalizationSubmap();

  // Shared closure processing (async — runs on background thread)
  vegvisir_.processLoopClosuresAsync(Vegvisir::QUERY_ID_LOCALIZATION,
                                     std::move(query_points_mc),
                                     std::move(query_points_icp), pose_odom_base);
}

std::vector<map_closures::ClosureCandidate>
LocalizationBackend::retrieveCandidates(
    int query_id, const std::vector<Eigen::Vector3d> &query_points_mc) {
  if (!vegvisir_.map_closer_) {
    return {};
  }
  // Localization: Query current map only (no add)
  return vegvisir_.map_closer_->QueryTopKClosures(query_id, query_points_mc, 1);
}

void LocalizationBackend::applyAcceptedClosure(
    const map_closures::ClosureCandidate &c,
    const Eigen::Matrix4d &query_odom_base) {
  handleClosureMeasurementUpdate(c.source_id, c.pose, query_odom_base);
}

void LocalizationBackend::handleClosureMeasurementUpdate(
    const int source_id, const Eigen::Matrix4d &pose,
    const Eigen::Matrix4d &query_odom_base) {
  // Get reference poses
  const auto &reference_poses = vegvisir_.getReferencePoses();
  if (reference_poses.empty()) {
    std::cerr << "No reference poses loaded! Cannot update global position."
              << std::endl;
    return;
  }

  // Get reference pose
  auto ref_it = reference_poses.find(source_id);
  if (ref_it == reference_poses.end()) {
    std::cerr << "Could not find reference pose for closure update."
              << std::endl;
    return;
  }

  const Eigen::Matrix4d &tf_map_ref = ref_it->second;

  const Eigen::Matrix4d &tf_local_query = query_odom_base;
  const Eigen::Matrix4d &pose_constraint = pose;

  // Compute constraint inverse
  Eigen::Matrix4d constraint_inv = pose_constraint.inverse();

  // Validate constraint
  double det_diff =
      std::abs(constraint_inv.block<3, 3>(0, 0).determinant() - 1.0);
  bool is_finite = constraint_inv.allFinite();

  // Reject constraint if either check fails
  if (!is_finite || det_diff > DETERMINANT_TOLERANCE) {
    return;
  }

  // Compute the measured map->odom transform from closure
  Eigen::Matrix4d measured_tf_map_odom =
      tf_map_ref * constraint_inv * tf_local_query.inverse();

  // Update Kalman filter with the measurement
  Sophus::SE3d Z_meas(measured_tf_map_odom);
  vegvisir_.pose_filter_.update(Z_meas);

  // Use the filtered estimate for tf_map_odom_
  Sophus::SE3d filtered_pose = vegvisir_.pose_filter_.state();
  vegvisir_.tf_map_odom_ = filtered_pose.matrix();
}

void LocalizationBackend::initLocalizationAnchor(
    const Eigen::Matrix4d &pose_odom_base) {
  if (localization_anchor_initialized_) {
    return;
  }
  localization_anchor_initialized_ = true;

  // Anchor starts at the first received odom pose
  pose_odom_anchor_ = pose_odom_base;

  // Reset/initialize the local_map_graph_ to represent the localization ring
  // buffer. One node (id 0) whose keypose is T_odom_anchor.
  vegvisir_.local_map_graph_.clear(0);
  vegvisir_.local_map_graph_.updateKeypose(vegvisir_.local_map_graph_.lastId(),
                                           pose_odom_anchor_);
  vegvisir_.local_map_graph_.lastLocalMap().clearTrajectory();

  // Start with a fresh voxel grid in this anchor frame
  vegvisir_.voxel_grid_.clear();

  vegvisir_.current_pose_ = Eigen::Matrix4d::Identity();
  vegvisir_.distance_since_query_ = 0.0;
}

void LocalizationBackend::pruneLocalizationSubmapBuffer() {
  // Keep only the most recent MAX_LOCALIZATION_SUBMAPS nodes
  while (static_cast<int>(vegvisir_.local_map_graph_.size()) >
         Vegvisir::MAX_LOCALIZATION_SUBMAPS) {
    uint64_t oldest_id = vegvisir_.local_map_graph_.begin()->first;
    uint64_t newest_id = vegvisir_.local_map_graph_.lastId();

    // Never delete the active node
    if (oldest_id == newest_id) {
      break;
    }
    vegvisir_.local_map_graph_.eraseLocalMap(oldest_id);
  }
}

void LocalizationBackend::cutLocalizationSubmap() {
  // Finalize the current active submap and create a new one whose keypose
  // equals endpose()

  uint64_t old_id = vegvisir_.local_map_graph_.lastId();

  // Ensure trajectory's "last" represents current_pose_ without growing
  // unbounded
  auto &traj = vegvisir_.local_map_graph_.lastLocalMap().localTrajectory();
  if (traj.empty()) {
    traj.push_back(vegvisir_.current_pose_);
  } else {
    traj.back() = vegvisir_.current_pose_;
  }

  // Store voxel_grid points into old map and create new map with keypose =
  // endpose()
  uint64_t new_id = vegvisir_.local_map_graph_.finalizeLocalMap(
      vegvisir_.voxel_grid_, Mode::LOCALIZATION);

  // Clear trajectories to save memory (ring buffer)
  vegvisir_.local_map_graph_[old_id].clearTrajectory();
  vegvisir_.local_map_graph_[new_id].clearTrajectory();

  // Start new active submap with a fresh voxel grid
  vegvisir_.voxel_grid_.clear();

  // Update active anchor from the newly created keypose (odom <- anchor_new)
  pose_odom_anchor_ = vegvisir_.local_map_graph_.lastKeypose();

  // Reset relative pose (anchor == base at cut time)
  vegvisir_.current_pose_ = Eigen::Matrix4d::Identity();

  // Enforce ring-buffer size
  pruneLocalizationSubmapBuffer();
}

void LocalizationBackend::buildLocalizationQueryCloudInBaseFrame(
    const Eigen::Matrix4d &pose_odom_base,
    std::vector<Eigen::Vector3d> &query_points_mc,
    std::vector<Eigen::Vector3d> &query_points_icp) const {

  query_points_mc.clear();
  query_points_icp.clear();

  // Transform from current anchor to current base:
  // T_base_anchor = (pose_odom_base)^-1 * (T_odom_anchor)
  // Note: current live voxel_grid points are stored in the active anchor
  // frame.
  const Eigen::Matrix4d T_base_anchor_cur =
      pose_odom_base.inverse() * pose_odom_anchor_;

  // 1) Add current voxel grid points
  const auto current_mc = vegvisir_.voxel_grid_.pointcloud();
  const auto [current_icp, current_normals] =
      vegvisir_.voxel_grid_.perVoxelPointAndNormal();

  Vegvisir::transformAndAppendPoints(current_mc, T_base_anchor_cur,
                                     query_points_mc);
  Vegvisir::transformAndAppendPoints(current_icp, T_base_anchor_cur,
                                     query_points_icp);

  // 2) Add stored submaps from ring buffer (their pointCloud is in their
  // anchor frame)
  for (auto it = vegvisir_.local_map_graph_.begin();
       it != vegvisir_.local_map_graph_.end(); ++it) {
    const LocalMap &submap = it->second;
    if (!submap.hasPointCloud()) {
      continue;
    }

    const Eigen::Matrix4d &pose_odom_anchor_i = submap.keypose();
    const Eigen::Matrix4d T_base_anchor_i =
        pose_odom_base.inverse() * pose_odom_anchor_i;

    Vegvisir::transformAndAppendPoints(submap.pointCloud(), T_base_anchor_i,
                                       query_points_mc);
    Vegvisir::transformAndAppendPoints(submap.pointCloud(), T_base_anchor_i,
                                       query_points_icp);
  }

  // Re-voxelize accumulated points to get consistent density
  query_points_mc =
      voxel_map::voxelDownsample(query_points_mc, Vegvisir::VOXEL_SIZE);
  query_points_icp =
      voxel_map::voxelDownsample(query_points_icp, Vegvisir::VOXEL_SIZE);
}

} // namespace vegvisir
