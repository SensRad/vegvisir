// Copyright (c) Sensrad 2026

#include "LocalizationBackend.hpp"

#include <iostream>

#include "Vegvisir.hpp"

namespace vegvisir {

LocalizationBackend::LocalizationBackend(Vegvisir& vegvisir) : VegvisirBackend(vegvisir) {}

void LocalizationBackend::initialize() {
  localization_anchor_initialized_ = false;
  pose_odom_anchor_.setIdentity();
}

void LocalizationBackend::updatePoseEstimate(const Eigen::Matrix4d& pose_odom_base,
                                             const Sophus::SE3d& delta_pose,
                                             uint64_t timestamp_ns) {
  // Localization: ring-buffer submaps
  initLocalizationAnchor(pose_odom_base, timestamp_ns);

  // current_pose_ is anchor <- base
  currentPose() = pose_odom_anchor_.inverse() * pose_odom_base;

  // Lock: pose_filter_ and tf_map_odom_ are written by background closure
  // thread
  const std::lock_guard<std::mutex> lock(closureMutex());
  poseFilter().predict(delta_pose);
  tfMapOdom() = poseFilter().state().matrix();
}

void LocalizationBackend::updateTrajectory() {
  // Don't grow trajectory unbounded in localization; store only the latest
  localMapGraph().lastLocalMap().localTrajectory().assign(1, currentPose());
}

double LocalizationBackend::queryDistanceM() const {
  return config().splitting_distance_localization;
}

void LocalizationBackend::runQueryCycle(const Eigen::Matrix4d& pose_odom_base,
                                        uint64_t timestamp_ns) {
  // Localization: build query cloud from ring buffer, then query closures
  std::vector<Eigen::Vector3d> query_points_mc;
  std::vector<Eigen::Vector3d> query_points_icp;

  buildLocalizationQueryCloudInBaseFrame(pose_odom_base, query_points_mc, query_points_icp);

  // Cut submap before async closure call (independent, must stay synchronous)
  cutLocalizationSubmap(timestamp_ns);

  // Shared closure processing (async — runs on background thread)
  processLoopClosuresAsync(Vegvisir::QUERY_ID_LOCALIZATION, std::move(query_points_mc),
                           std::move(query_points_icp), pose_odom_base);
}

std::vector<map_closures::ClosureCandidate> LocalizationBackend::retrieveCandidates(
    int query_id, const std::vector<Eigen::Vector3d>& query_points_mc) {
  if (!mapCloser()) {
    return {};
  }
  // Localization: Query current map only (no add)
  return mapCloser()->queryTopKClosures(query_id, query_points_mc, 1);
}

void LocalizationBackend::applyAcceptedClosure(const map_closures::ClosureCandidate& c,
                                               const Eigen::Matrix4d& query_odom_base) {
  handleClosureMeasurementUpdate(c.source_id, c.pose, query_odom_base);
}

void LocalizationBackend::handleClosureMeasurementUpdate(const int source_id,
                                                         const Eigen::Matrix4d& pose,
                                                         const Eigen::Matrix4d& query_odom_base) {
  // Get reference poses
  const auto& ref_poses = referencePoses();
  if (ref_poses.empty()) {
    std::cerr << "No reference poses loaded! Cannot update global position." << '\n';
    return;
  }

  // Get reference pose
  auto ref_it = ref_poses.find(source_id);
  if (ref_it == ref_poses.end()) {
    std::cerr << "Could not find reference pose for closure update." << '\n';
    return;
  }

  const Eigen::Matrix4d& tf_map_ref = ref_it->second;

  const Eigen::Matrix4d& tf_local_query = query_odom_base;
  const Eigen::Matrix4d& pose_constraint = pose;

  // Compute constraint inverse
  Eigen::Matrix4d constraint_inv = pose_constraint.inverse();

  // Validate constraint
  const double det_diff = std::abs(constraint_inv.block<3, 3>(0, 0).determinant() - 1.0);
  const bool is_finite = constraint_inv.allFinite();

  // Reject constraint if either check fails
  if (!is_finite || det_diff > DETERMINANT_TOLERANCE) {
    return;
  }

  // Compute the measured map->odom transform from closure
  const Eigen::Matrix4d measured_tf_map_odom =
      tf_map_ref * constraint_inv * tf_local_query.inverse();

  // Update Kalman filter with the measurement
  const Sophus::SE3d z_meas(measured_tf_map_odom);
  poseFilter().update(z_meas);

  // Use the filtered estimate for tf_map_odom_
  const Sophus::SE3d filtered_pose = poseFilter().state();
  tfMapOdom() = filtered_pose.matrix();
}

void LocalizationBackend::initLocalizationAnchor(const Eigen::Matrix4d& pose_odom_base,
                                                 uint64_t timestamp_ns) {
  if (localization_anchor_initialized_) {
    return;
  }
  localization_anchor_initialized_ = true;

  // Anchor starts at the first received odom pose
  pose_odom_anchor_ = pose_odom_base;

  // Reset/initialize the local_map_graph_ to represent the localization ring
  // buffer. One node (id 0) whose keypose is T_odom_anchor.
  localMapGraph().clear(0, timestamp_ns);
  localMapGraph().updateKeypose(localMapGraph().lastId(), pose_odom_anchor_);
  localMapGraph().lastLocalMap().clearTrajectory();

  // Start with a fresh voxel grid in this anchor frame
  voxelGrid().clear();

  currentPose() = Eigen::Matrix4d::Identity();
  distanceSinceQuery() = 0.0;
}

void LocalizationBackend::pruneLocalizationSubmapBuffer() {
  // Keep only the most recent submaps (derived from splitting distances)
  while (static_cast<int>(localMapGraph().size()) > maxLocalizationSubmaps()) {
    const uint64_t oldest_id = localMapGraph().begin()->first;
    const uint64_t newest_id = localMapGraph().lastId();

    // Never delete the active node
    if (oldest_id == newest_id) {
      break;
    }
    localMapGraph().eraseLocalMap(oldest_id);
  }
}

void LocalizationBackend::cutLocalizationSubmap(uint64_t timestamp_ns) {
  // Finalize the current active submap and create a new one whose keypose
  // equals endpose()

  const uint64_t old_id = localMapGraph().lastId();

  // Ensure trajectory's "last" represents current_pose_ without growing
  // unbounded
  auto& traj = localMapGraph().lastLocalMap().localTrajectory();
  if (traj.empty()) {
    traj.push_back(currentPose());
  } else {
    traj.back() = currentPose();
  }

  // Store voxel_grid points into old map and create new map with keypose =
  // endpose()
  const uint64_t new_id =
      localMapGraph().finalizeLocalMap(voxelGrid(), Mode::LOCALIZATION, timestamp_ns);

  // Clear trajectories to save memory (ring buffer)
  localMapGraph()[old_id].clearTrajectory();
  localMapGraph()[new_id].clearTrajectory();

  // Start new active submap with a fresh voxel grid
  voxelGrid().clear();

  // Update active anchor from the newly created keypose (odom <- anchor_new)
  pose_odom_anchor_ = localMapGraph().lastKeypose();

  // Reset relative pose (anchor == base at cut time)
  currentPose() = Eigen::Matrix4d::Identity();

  // Enforce ring-buffer size
  pruneLocalizationSubmapBuffer();
}

void LocalizationBackend::buildLocalizationQueryCloudInBaseFrame(
    const Eigen::Matrix4d& pose_odom_base, std::vector<Eigen::Vector3d>& query_points_mc,
    std::vector<Eigen::Vector3d>& query_points_icp) {
  query_points_mc.clear();
  query_points_icp.clear();

  // Transform from current anchor to current base:
  // T_base_anchor = (pose_odom_base)^-1 * (T_odom_anchor)
  // Note: current live voxel_grid points are stored in the active anchor
  // frame.
  const Eigen::Matrix4d t_base_anchor_cur = pose_odom_base.inverse() * pose_odom_anchor_;

  // 1) Add current voxel grid points
  const auto current_mc = voxelGrid().pointcloud();
  const auto [current_icp, current_normals] = voxelGrid().perVoxelPointAndNormal();

  Vegvisir::transformAndAppendPoints(current_mc, t_base_anchor_cur, query_points_mc);
  Vegvisir::transformAndAppendPoints(current_icp, t_base_anchor_cur, query_points_icp);

  // 2) Add stored submaps from ring buffer (their pointCloud is in their
  // anchor frame)
  for (const auto& [key, submap_val] : localMapGraph()) {
    const LocalMap& submap = submap_val;
    if (!submap.hasPointCloud()) {
      continue;
    }

    const Eigen::Matrix4d& pose_odom_anchor_i = submap.keypose();
    const Eigen::Matrix4d t_base_anchor_i = pose_odom_base.inverse() * pose_odom_anchor_i;

    Vegvisir::transformAndAppendPoints(submap.pointCloud(), t_base_anchor_i, query_points_mc);
    Vegvisir::transformAndAppendPoints(submap.pointCloud(), t_base_anchor_i, query_points_icp);
  }

  // Re-voxelize accumulated points to get consistent density
  query_points_mc = voxel_map::voxelDownsample(query_points_mc, config().voxel_size);
  query_points_icp = voxel_map::voxelDownsample(query_points_icp, config().voxel_size);
}

}  // namespace vegvisir
