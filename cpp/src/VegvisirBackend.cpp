// Copyright (c) Sensrad 2026

#include "VegvisirBackend.hpp"

#include "Vegvisir.hpp"

namespace vegvisir {

LocalMapGraph& VegvisirBackend::localMapGraph() {
  return vegvisir_.local_map_graph_;
}

voxel_map::VoxelMap& VegvisirBackend::voxelGrid() {
  return vegvisir_.voxel_grid_;
}

Eigen::Matrix4d& VegvisirBackend::currentPose() {
  return vegvisir_.current_pose_;
}

Eigen::Matrix4d& VegvisirBackend::tfMapOdom() {
  return vegvisir_.tf_map_odom_;
}

double& VegvisirBackend::distanceSinceQuery() {
  return vegvisir_.distance_since_query_;
}

kalman_filter::PoseKalmanFilter& VegvisirBackend::poseFilter() {
  return vegvisir_.pose_filter_;
}

std::mutex& VegvisirBackend::closureMutex() {
  return vegvisir_.closure_mutex_;
}

map_closures::MapClosures *VegvisirBackend::mapCloser() {
  return vegvisir_.map_closer_.get();
}

std::unordered_map<int, std::vector<Eigen::Vector3d>>& VegvisirBackend::localMapPoints() {
  return vegvisir_.local_map_points_;
}

const VegvisirConfig& VegvisirBackend::config() const {
  return vegvisir_.config_;
}

const std::unordered_map<int, Eigen::Matrix4d>& VegvisirBackend::referencePoses() const {
  return vegvisir_.getReferencePoses();
}

void VegvisirBackend::processLoopClosuresAsync(int query_id,
                                               std::vector<Eigen::Vector3d> query_points_mc,
                                               std::vector<Eigen::Vector3d> query_points_icp,
                                               Eigen::Matrix4d query_odom_base) {
  vegvisir_.processLoopClosuresAsync(query_id, std::move(query_points_mc),
                                     std::move(query_points_icp), query_odom_base);
}

}  // namespace vegvisir
