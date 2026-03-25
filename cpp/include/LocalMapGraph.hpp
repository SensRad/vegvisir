// Copyright (c) Sensrad 2026

#pragma once

#include <cstdint>

#include <fstream>
#include <map>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

#include "voxel_map/VoxelMap.hpp"

namespace vegvisir {

// Forward declaration of Mode enum (defined in Vegvisir.hpp)
enum class Mode : std::uint8_t;

/**
 * Represents a single local map in the pose graph.
 *
 * Each LocalMap corresponds to a segment of the trajectory, starting at a
 * keypose. The local_trajectory stores relative poses within this segment.
 * When the local map is finalized, the point cloud is stored in pcd.
 */
class LocalMap {
 public:
  LocalMap(uint64_t id, Eigen::Matrix4d keypose);

  // Getters
  [[nodiscard]] uint64_t id() const { return id_; }
  [[nodiscard]] const Eigen::Matrix4d& keypose() const { return keypose_; }
  Eigen::Matrix4d& keypose() { return keypose_; }

  [[nodiscard]] const std::vector<Eigen::Matrix4d>& localTrajectory() const {
    return local_trajectory_;
  }
  std::vector<Eigen::Matrix4d>& localTrajectory() { return local_trajectory_; }

  [[nodiscard]] const std::vector<Eigen::Vector3d>& pointCloud() const { return pcd_; }
  std::vector<Eigen::Vector3d>& pointCloud() { return pcd_; }

  [[nodiscard]] Eigen::Matrix4d endpose() const;

  void addToTrajectory(const Eigen::Matrix4d& relative_pose);
  void clearTrajectory();
  [[nodiscard]] bool write(const std::string& filename) const;
  [[nodiscard]] bool hasPointCloud() const { return !pcd_.empty(); }

 private:
  uint64_t id_;
  Eigen::Matrix4d keypose_;
  std::vector<Eigen::Matrix4d> local_trajectory_;
  std::vector<Eigen::Vector3d> pcd_;
};

class LocalMapGraph {
 public:
  explicit LocalMapGraph(int initial_map_id = 0);

  // Iterator types for range-based for loops
  using MapIterator = std::map<uint64_t, LocalMap>::iterator;
  using ConstMapIterator = std::map<uint64_t, LocalMap>::const_iterator;

  LocalMap& operator[](uint64_t key);
  const LocalMap& operator[](uint64_t key) const;

  [[nodiscard]] bool hasLocalMap(uint64_t key) const;
  [[nodiscard]] size_t size() const { return graph_.size(); }
  [[nodiscard]] bool empty() const { return graph_.empty(); }

  // Iteration support
  MapIterator begin() { return graph_.begin(); }
  MapIterator end() { return graph_.end(); }
  [[nodiscard]] ConstMapIterator begin() const { return graph_.begin(); }
  [[nodiscard]] ConstMapIterator end() const { return graph_.end(); }
  [[nodiscard]] ConstMapIterator cbegin() const { return graph_.cbegin(); }
  [[nodiscard]] ConstMapIterator cend() const { return graph_.cend(); }

  [[nodiscard]] uint64_t lastId() const;

  LocalMap& lastLocalMap();
  [[nodiscard]] const LocalMap& lastLocalMap() const;

  [[nodiscard]] const Eigen::Matrix4d& lastKeypose() const;
  Eigen::Matrix4d& lastKeypose();

  void eraseLocalMap(uint64_t key);
  void eraseLastLocalMap();

  uint64_t finalizeLocalMap(voxel_map::VoxelMap& voxel_grid, Mode mode);
  uint64_t finalizeLocalMap();

  void setPointCloud(uint64_t key, const std::vector<Eigen::Vector3d>& points);

  [[nodiscard]] std::vector<Eigen::Matrix4d> getAllKeyposes() const;
  [[nodiscard]] std::vector<uint64_t> getAllIds() const;

  void updateKeypose(uint64_t key, const Eigen::Matrix4d& new_keypose);
  void addLocalMap(uint64_t id, const Eigen::Matrix4d& keypose);
  void clear(int initial_map_id = 0);

 private:
  std::map<uint64_t, LocalMap> graph_;
};

}  // namespace vegvisir
