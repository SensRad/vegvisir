// Copyright (c) Sensrad 2025-2026

#include "SlamBackend.hpp"
#include "Vegvisir.hpp"

#include <iostream>

namespace vegvisir {

SlamBackend::SlamBackend(Vegvisir &vegvisir) : VegvisirBackend(vegvisir) {}

void SlamBackend::initialize() {
  pose_at_nodes_.clear();
  pose_at_nodes_.push_back(Eigen::Matrix4d::Identity());

  const int max_iterations = 10;
  keypose_optimizer_ =
      std::make_unique<pgo::PoseGraphOptimizer>(max_iterations);

  // If the graph is empty, create new variable for the first keypose and fix it
  if (vegvisir_.local_map_graph_.empty()) {
    keypose_optimizer_->addVariable(vegvisir_.local_map_graph_.lastId(),
                                    vegvisir_.local_map_graph_.lastKeypose());
    keypose_optimizer_->fixVariable(vegvisir_.local_map_graph_.lastId());
    return;
  }

  // If we have loaded reference poses, use them to decide which variables to
  // fix. reference_poses maps map_id -> pose and is non-empty when a DB was
  // loaded.
  const auto &reference_poses = vegvisir_.getReferencePoses();

  for (const auto &[map_id, local_map] : vegvisir_.local_map_graph_) {
    keypose_optimizer_->addVariable(map_id, local_map.keypose());

    // If this map_id exists in the loaded reference poses, fix it.
    // That prevents the optimizer from moving the pre-built map.
    if (!reference_poses.empty() && reference_poses.find(static_cast<int>(
                                        map_id)) != reference_poses.end()) {
      keypose_optimizer_->fixVariable(map_id);
    }
  }

  // If no reference poses exist (i.e., started from scratch), fix the first
  // keypose so graph has a stable anchor.
  if (reference_poses.empty()) {
    if (!vegvisir_.local_map_graph_.getAllIds().empty()) {
      keypose_optimizer_->fixVariable(
          vegvisir_.local_map_graph_.getAllIds().front());
    }
  } else {
    // Reference poses exist (loaded a map). Create a new "current trajectory"
    // node at identity. This node is unfixed and initially disconnected from
    // the reference map - it will be connected when a closure is found.
    uint64_t new_id = vegvisir_.local_map_graph_.lastId() + 1;
    vegvisir_.local_map_graph_.addLocalMap(new_id, Eigen::Matrix4d::Identity());
    keypose_optimizer_->addVariable(new_id, Eigen::Matrix4d::Identity());

    std::cout << "Created new trajectory starting node " << new_id
              << " at identity (unfixed)" << std::endl;
  }

  std::cout << "SLAM backend initialized optimizer with "
            << vegvisir_.local_map_graph_.size() << " keyposes" << std::endl;
}

void SlamBackend::preIntegrate(const Eigen::Matrix4d &pose_odom_base,
                               const Sophus::SE3d & /*delta_pose*/) {
  // Compensate for poses at previous nodes
  Eigen::Matrix4d compensated_pose = pose_odom_base;
  for (const auto &pose : pose_at_nodes_) {
    compensated_pose = pose.inverse() * compensated_pose;
  }
  vegvisir_.current_pose_ = compensated_pose;

  // Lock: reads lastKeypose() (written by background optimizeKeyposeGraph)
  std::lock_guard<std::mutex> lock(vegvisir_.closure_mutex_);
  Eigen::Matrix4d T_map_base =
      vegvisir_.local_map_graph_.lastKeypose() * vegvisir_.current_pose_;
  vegvisir_.tf_map_odom_ = T_map_base * pose_odom_base.inverse();
}

void SlamBackend::postIntegrate() {
  // Append compensated pose to the current local map's trajectory
  vegvisir_.local_map_graph_.lastLocalMap().addToTrajectory(
      vegvisir_.current_pose_);
}

double SlamBackend::queryDistanceM() const {
  return Vegvisir::QUERY_DISTANCE_SLAM_M;
}

void SlamBackend::runQueryCycle(const Eigen::Matrix4d &pose_odom_base) {
  // In SLAM mode, generate a new node after each query (and handle closures)
  generateNewNode(pose_odom_base);
}

std::vector<map_closures::ClosureCandidate> SlamBackend::retrieveCandidates(
    int query_id, const std::vector<Eigen::Vector3d> &query_points_mc) {
  if (!vegvisir_.map_closer_) {
    return {};
  }
  // SLAM: Query current map and add to database (query + add)
  return vegvisir_.map_closer_->GetTopKClosures(query_id, query_points_mc, 1);
}

void SlamBackend::applyAcceptedClosure(
    const map_closures::ClosureCandidate &c,
    const Eigen::Matrix4d & /*query_odom_base*/) {
  if (!keypose_optimizer_) {
    return;
  }

  Eigen::Matrix<double, 6, 6> information =
      Eigen::Matrix<double, 6, 6>::Identity();

  keypose_optimizer_->addFactor(c.source_id, c.target_id, c.pose, information);

  // Optimize keypose graph immediately after adding closure
  optimizeKeyposeGraph();
}

void SlamBackend::optimizeKeyposeGraph() {
  if (!keypose_optimizer_) {
    return;
  }

  keypose_optimizer_->optimize();
  auto estimates = keypose_optimizer_->estimates();

  for (const auto &[map_id, local_map] : vegvisir_.local_map_graph_) {
    auto it = estimates.find(static_cast<int>(map_id));
    if (it != estimates.end()) {
      vegvisir_.local_map_graph_.updateKeypose(map_id, it->second);
    }
  }
}

void SlamBackend::generateNewNode(const Eigen::Matrix4d &pose_odom_base) {
  // Generate a new SLAM node and check for loop closures

  LocalMap &last_local_map = vegvisir_.local_map_graph_.lastLocalMap();

  // Get the relative motion (last element of local trajectory)
  const auto &local_trajectory = last_local_map.localTrajectory();
  if (local_trajectory.empty()) {
    std::cerr << "generateNewNode: local trajectory is empty, skipping"
              << std::endl;
    return;
  }
  Eigen::Matrix4d relative_motion = local_trajectory.back();
  Eigen::Matrix4d inverse_relative_motion = relative_motion.inverse();

  // Get points from voxel grid and transform by inverse relative motion
  std::vector<Eigen::Vector3d> points = vegvisir_.voxel_grid_.pointcloud();
  std::vector<Eigen::Vector3d> transformed_points;
  transformed_points.reserve(points.size());
  for (const auto &p : points) {
    Eigen::Vector4d hp(p.x(), p.y(), p.z(), 1.0);
    Eigen::Vector4d transformed = inverse_relative_motion * hp;
    transformed_points.emplace_back(transformed.x(), transformed.y(),
                                    transformed.z());
  }

  // Log the pose of the last node
  pose_at_nodes_.push_back(relative_motion);

  // Store query ID and get query points before finalizing
  uint64_t query_id_u64 = last_local_map.id();
  int query_id = static_cast<int>(query_id_u64);

  auto query_points_mc = vegvisir_.voxel_grid_.pointcloud();
  auto [query_points_icp, _] = vegvisir_.voxel_grid_.perVoxelPointAndNormal();

  {
    // Lock: serializes with background optimizeKeyposeGraph which accesses
    // keypose_optimizer_, local_map_graph_.graph_, and local_map_points_
    std::lock_guard<std::mutex> lock(vegvisir_.closure_mutex_);

    // Store local map points for ICP refinement
    vegvisir_.local_map_points_[query_id] = query_points_icp;

    // Finalize the local map and create a new one
    uint64_t new_id = vegvisir_.local_map_graph_.finalizeLocalMap(
        vegvisir_.voxel_grid_, Mode::SLAM);

    // Add variable to optimizer for the new keypose + odometry factor
    if (keypose_optimizer_) {
      keypose_optimizer_->addVariable(new_id,
                                      vegvisir_.local_map_graph_.lastKeypose());

      Eigen::Matrix<double, 6, 6> information =
          Eigen::Matrix<double, 6, 6>::Identity();

      keypose_optimizer_->addFactor(new_id, query_id_u64, relative_motion,
                                    information);
    }
  }

  // Voxel grid is main-thread only — no lock needed
  vegvisir_.voxel_grid_.clear();
  vegvisir_.voxel_grid_.addPoints(transformed_points);

  // Shared closure processing (async — runs on background thread)
  vegvisir_.processLoopClosuresAsync(query_id, std::move(query_points_mc),
                                     std::move(query_points_icp), pose_odom_base);
}

} // namespace vegvisir
