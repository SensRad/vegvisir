// Copyright (c) Sensrad 2025-2026

#include "LocalMapGraph.hpp"
#include "Vegvisir.hpp" // For Mode enum definition

#include <utility>

namespace vegvisir {

// ============================================================================
// LocalMap Implementation
// ============================================================================

LocalMap::LocalMap(uint64_t id, Eigen::Matrix4d keypose)
    : id_(id), keypose_(std::move(keypose)) {
  // Initialize with empty trajectory
  // The trajectory will be populated as poses are added
}

Eigen::Matrix4d LocalMap::endpose() const {
  if (local_trajectory_.empty()) {
    return keypose_;
  }
  return keypose_ * local_trajectory_.back();
}

void LocalMap::addToTrajectory(const Eigen::Matrix4d &relative_pose) {
  local_trajectory_.push_back(relative_pose);
}

void LocalMap::clearTrajectory() { local_trajectory_.clear(); }

bool LocalMap::write(const std::string &filename) const {
  std::ofstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    return false;
  }

  // Transform points by keypose before writing
  for (const auto &point : pcd_) {
    const Eigen::Vector4d p_hom(point.x(), point.y(), point.z(), 1.0);
    Eigen::Vector4d p_transformed = keypose_ * p_hom;

    auto x = static_cast<float>(p_transformed.x());
    auto y = static_cast<float>(p_transformed.y());
    auto z = static_cast<float>(p_transformed.z());

    file.write(reinterpret_cast<const char *>(&x), sizeof(float));
    file.write(reinterpret_cast<const char *>(&y), sizeof(float));
    file.write(reinterpret_cast<const char *>(&z), sizeof(float));
  }

  return file.good();
}

// ============================================================================
// LocalMapGraph Implementation
// ============================================================================

LocalMapGraph::LocalMapGraph(int initial_map_id) {
  // Create the initial local map with identity keypose
  LocalMap initial_map(static_cast<uint64_t>(initial_map_id),
                       Eigen::Matrix4d::Identity());
  // Don't add Identity to trajectory - start with empty trajectory
  graph_.emplace(static_cast<uint64_t>(initial_map_id), std::move(initial_map));
}

LocalMap &LocalMapGraph::operator[](uint64_t key) {
  auto it = graph_.find(key);
  if (it == graph_.end()) {
    throw std::out_of_range("LocalMapGraph: key " + std::to_string(key) +
                            " not found");
  }
  return it->second;
}

const LocalMap &LocalMapGraph::operator[](uint64_t key) const {
  auto it = graph_.find(key);
  if (it == graph_.end()) {
    throw std::out_of_range("LocalMapGraph: key " + std::to_string(key) +
                            " not found");
  }
  return it->second;
}

bool LocalMapGraph::hasLocalMap(uint64_t key) const {
  return graph_.find(key) != graph_.end();
}

uint64_t LocalMapGraph::lastId() const {
  if (graph_.empty()) {
    throw std::runtime_error("LocalMapGraph is empty");
  }
  return graph_.rbegin()->first;
}

LocalMap &LocalMapGraph::lastLocalMap() { return graph_.rbegin()->second; }

const LocalMap &LocalMapGraph::lastLocalMap() const {
  return graph_.rbegin()->second;
}

const Eigen::Matrix4d &LocalMapGraph::lastKeypose() const {
  return lastLocalMap().keypose();
}

Eigen::Matrix4d &LocalMapGraph::lastKeypose() {
  return lastLocalMap().keypose();
}

void LocalMapGraph::eraseLocalMap(uint64_t key) { graph_.erase(key); }

void LocalMapGraph::eraseLastLocalMap() {
  if (!graph_.empty()) {
    graph_.erase(std::prev(graph_.end()));
  }
}

uint64_t LocalMapGraph::finalizeLocalMap(voxel_map::VoxelMap &voxel_grid,
                                         Mode mode) {
  LocalMap &current_map = lastLocalMap();

  if (mode == Mode::LOCALIZATION) {
    // In localization mode, store the full point cloud
    current_map.pointCloud() = voxel_grid.pointcloud();
  } else {
    // In SLAM mode, store only per-voxel points for efficiency
    auto [query_points_icp, query_normals] =
        voxel_grid.perVoxelPointAndNormal();
    current_map.pointCloud() = query_points_icp;
  }

  // Compute the new keypose (endpose of current map)
  uint64_t new_id = current_map.id() + 1;
  const Eigen::Matrix4d new_keypose = current_map.endpose();

  // Create the new local map
  LocalMap new_map(new_id, new_keypose);

  // Start with Identity in trajectory
  // This is important for fine-grained optimization which iterates over
  // local_trajectory[1:] to skip the keypose itself
  new_map.addToTrajectory(Eigen::Matrix4d::Identity());

  graph_.emplace(new_id, std::move(new_map));

  return new_id;
}

uint64_t LocalMapGraph::finalizeLocalMap() {
  const LocalMap &current_map = lastLocalMap();

  // Compute the new keypose (endpose of current map)
  uint64_t new_id = current_map.id() + 1;
  const Eigen::Matrix4d new_keypose = current_map.endpose();

  // Create the new local map
  LocalMap new_map(new_id, new_keypose);

  // Start with Identity in trajectory
  new_map.addToTrajectory(Eigen::Matrix4d::Identity());

  graph_.emplace(new_id, std::move(new_map));

  return new_id;
}

void LocalMapGraph::setPointCloud(uint64_t key,
                                  const std::vector<Eigen::Vector3d> &points) {
  (*this)[key].pointCloud() = points;
}

std::vector<Eigen::Matrix4d> LocalMapGraph::getAllKeyposes() const {
  std::vector<Eigen::Matrix4d> keyposes;
  keyposes.reserve(graph_.size());
  for (const auto &[id, local_map] : graph_) {
    keyposes.push_back(local_map.keypose());
  }
  return keyposes;
}

std::vector<uint64_t> LocalMapGraph::getAllIds() const {
  std::vector<uint64_t> ids;
  ids.reserve(graph_.size());
  for (const auto &[id, local_map] : graph_) {
    ids.push_back(id);
  }
  return ids;
}

void LocalMapGraph::updateKeypose(uint64_t key,
                                  const Eigen::Matrix4d &new_keypose) {
  (*this)[key].keypose() = new_keypose;
}

void LocalMapGraph::addLocalMap(uint64_t id, const Eigen::Matrix4d &keypose) {
  LocalMap new_map(id, keypose);
  graph_.emplace(id, std::move(new_map));
}

void LocalMapGraph::clear(int initial_map_id) {
  graph_.clear();
  LocalMap initial_map(static_cast<uint64_t>(initial_map_id),
                       Eigen::Matrix4d::Identity());
  graph_.emplace(static_cast<uint64_t>(initial_map_id), std::move(initial_map));
}

} // namespace vegvisir
