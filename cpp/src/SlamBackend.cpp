// Copyright (c) Sensrad 2026

#include "SlamBackend.hpp"

#include <iostream>

#include "Vegvisir.hpp"

namespace vegvisir {

SlamBackend::SlamBackend(Vegvisir& vegvisir) : VegvisirBackend(vegvisir) {}

void SlamBackend::initialize() {
  pose_at_nodes_.clear();
  pose_at_nodes_.emplace_back(Eigen::Matrix4d::Identity());

  keypose_optimizer_ = std::make_unique<pgo::PoseGraphOptimizer>(config().pgo_max_iterations);

  // If the graph is empty, create new variable for the first keypose and fix it
  if (localMapGraph().empty()) {
    keypose_optimizer_->addVariable(static_cast<int>(localMapGraph().lastId()),
                                    localMapGraph().lastKeypose());
    keypose_optimizer_->fixVariable(static_cast<int>(localMapGraph().lastId()));
    return;
  }

  // If we have loaded reference poses, use them to decide which variables to
  // fix. reference_poses maps map_id -> pose and is non-empty when a DB was
  // loaded.
  const auto& ref_poses = referencePoses();

  for (const auto& [map_id, local_map] : localMapGraph()) {
    keypose_optimizer_->addVariable(static_cast<int>(map_id), local_map.keypose());

    // If this map_id exists in the loaded reference poses, fix it.
    // That prevents the optimizer from moving the pre-built map.
    if (!ref_poses.empty() && ref_poses.find(static_cast<int>(map_id)) != ref_poses.end()) {
      keypose_optimizer_->fixVariable(static_cast<int>(map_id));
    }
  }

  // If no reference poses exist (i.e., started from scratch), fix the first
  // keypose so graph has a stable anchor.
  if (ref_poses.empty()) {
    if (!localMapGraph().getAllIds().empty()) {
      keypose_optimizer_->fixVariable(static_cast<int>(localMapGraph().getAllIds().front()));
    }
  } else {
    // Reference poses exist (loaded a map). Create a new "current trajectory"
    // node at identity. This node is unfixed and initially disconnected from
    // the reference map - it will be connected when a closure is found.
    const uint64_t new_id = localMapGraph().lastId() + 1;
    localMapGraph().addLocalMap(new_id, Eigen::Matrix4d::Identity());
    keypose_optimizer_->addVariable(static_cast<int>(new_id), Eigen::Matrix4d::Identity());

    std::cout << "Created new trajectory starting node " << new_id << " at identity (unfixed)"
              << '\n';
  }

  std::cout << "SLAM backend initialized optimizer with " << localMapGraph().size() << " keyposes"
            << '\n';
}

void SlamBackend::updatePoseEstimate(const Eigen::Matrix4d& pose_odom_base,
                                     const Sophus::SE3d& /*delta_pose*/,
                                     uint64_t /*timestamp_ns*/) {
  // Compensate for poses at previous nodes
  Eigen::Matrix4d compensated_pose = pose_odom_base;
  for (const auto& pose : pose_at_nodes_) {
    compensated_pose = pose.inverse() * compensated_pose;
  }
  currentPose() = compensated_pose;

  // Lock: reads lastKeypose() (written by background optimizeKeyposeGraph)
  const std::lock_guard<std::mutex> lock(closureMutex());
  const Eigen::Matrix4d t_map_base = localMapGraph().lastKeypose() * currentPose();
  tfMapOdom() = t_map_base * pose_odom_base.inverse();
}

void SlamBackend::updateTrajectory() {
  // Append compensated pose to the current local map's trajectory
  localMapGraph().lastLocalMap().addToTrajectory(currentPose());
}

double SlamBackend::queryDistanceM() const {
  return config().splitting_distance_slam;
}

void SlamBackend::runQueryCycle(const Eigen::Matrix4d& pose_odom_base, uint64_t timestamp_ns) {
  // In SLAM mode, generate a new node after each query (and handle closures)
  generateNewNode(pose_odom_base, timestamp_ns);
}

std::vector<map_closures::ClosureCandidate> SlamBackend::retrieveCandidates(
    int query_id, const std::vector<Eigen::Vector3d>& query_points_mc) {
  if (!mapCloser()) {
    return {};
  }
  // SLAM: Query current map and add to database (query + add)
  return mapCloser()->getTopKClosures(query_id, query_points_mc, 1);
}

void SlamBackend::applyAcceptedClosure(const map_closures::ClosureCandidate& c,
                                       const Eigen::Matrix4d& /*query_odom_base*/) {
  if (!keypose_optimizer_) {
    return;
  }

  const Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();

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

  for (const auto& [map_id, local_map] : localMapGraph()) {
    auto it = estimates.find(static_cast<int>(map_id));
    if (it != estimates.end()) {
      const Eigen::Matrix4d& new_keypose = it->second;
      localMapGraph().updateKeypose(map_id, new_keypose);

      // Update MapClosures reference pose to stay consistent
      if (mapCloser()) {
        mapCloser()->setReferencePose(static_cast<int>(map_id), new_keypose);
      }

      // Point clouds in pcd_ and local_map_points_ are in keypose-local
      // frame. Do NOT re-transform them: the updated keypose automatically
      // gives the corrected world position via keypose * p_local.
    }
  }
}

void SlamBackend::generateNewNode(const Eigen::Matrix4d& pose_odom_base, uint64_t timestamp_ns) {
  // Generate a new SLAM node and check for loop closures

  LocalMap& last_local_map = localMapGraph().lastLocalMap();

  // Get the relative motion (last element of local trajectory)
  const auto& local_trajectory = last_local_map.localTrajectory();
  if (local_trajectory.empty()) {
    std::cerr << "generateNewNode: local trajectory is empty, skipping" << '\n';
    return;
  }
  const Eigen::Matrix4d relative_motion = local_trajectory.back();
  const Eigen::Matrix4d inverse_relative_motion = relative_motion.inverse();

  // Get points from voxel grid and transform by inverse relative motion
  const std::vector<Eigen::Vector3d> points = voxelGrid().pointcloud();
  std::vector<Eigen::Vector3d> transformed_points;
  transformed_points.reserve(points.size());
  for (const auto& p : points) {
    const Eigen::Vector4d hp(p.x(), p.y(), p.z(), 1.0);
    const Eigen::Vector4d transformed = inverse_relative_motion * hp;
    transformed_points.emplace_back(transformed.x(), transformed.y(), transformed.z());
  }

  // Log the pose of the last node
  pose_at_nodes_.push_back(relative_motion);

  // Store query ID and get query points before finalizing
  const uint64_t query_id_u64 = last_local_map.id();
  const int query_id = static_cast<int>(query_id_u64);

  // Extract point for MapClosures and ICP refinement
  auto query_points_mc = voxelGrid().pointcloud();
  auto [query_points_icp, _] = voxelGrid().perVoxelPointAndNormal();

  {
    // Lock: serializes with background optimizeKeyposeGraph which accesses
    // keypose_optimizer_, local_map_graph_.graph_, and local_map_points_
    const std::lock_guard<std::mutex> lock(closureMutex());

    // Store local map points for ICP refinement
    localMapPoints()[query_id] = query_points_icp;

    // Finalize the local map and create a new one.
    const uint64_t new_id = localMapGraph().finalizeLocalMap(voxelGrid(), Mode::SLAM, timestamp_ns);

    // Add variable to optimizer for the new keypose + odometry factor
    if (keypose_optimizer_) {
      keypose_optimizer_->addVariable(static_cast<int>(new_id), localMapGraph().lastKeypose());

      const Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();

      keypose_optimizer_->addFactor(static_cast<int>(new_id), static_cast<int>(query_id_u64),
                                    relative_motion, information);
    }
  }

  // Voxel grid is main-thread only — no lock needed
  voxelGrid().clear();
  voxelGrid().addPoints(transformed_points);

  // Shared closure processing (async — runs on background thread)
  processLoopClosuresAsync(query_id, std::move(query_points_mc), std::move(query_points_icp),
                           pose_odom_base);
}

}  // namespace vegvisir
