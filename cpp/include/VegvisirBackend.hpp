// Copyright (c) Sensrad 2026

#pragma once

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "map_closures/MapClosures.hpp"

namespace voxel_map {
class VoxelMap;
}

namespace vegvisir {

// Forward declarations
class Vegvisir;
class LocalMapGraph;
}  // namespace vegvisir

namespace kalman_filter {
class PoseKalmanFilter;
}

namespace vegvisir {
struct VegvisirConfig;

// Backend interface (abstract base class for SLAM and Localization modes)
class VegvisirBackend {
 public:
  explicit VegvisirBackend(Vegvisir& vegvisir) : vegvisir_(vegvisir) {}
  virtual ~VegvisirBackend() = default;
  VegvisirBackend(const VegvisirBackend&) = delete;
  VegvisirBackend& operator=(const VegvisirBackend&) = delete;
  VegvisirBackend(VegvisirBackend&&) = default;
  VegvisirBackend& operator=(VegvisirBackend&&) = default;

  virtual void initialize() = 0;

  virtual void updatePoseEstimate(const Eigen::Matrix4d& pose_odom_base,
                                  const Sophus::SE3d& delta_pose) = 0;

  virtual void updateTrajectory() = 0;

  [[nodiscard]] virtual double queryDistanceM() const = 0;

  virtual void runQueryCycle(const Eigen::Matrix4d& pose_odom_base) = 0;

  virtual std::vector<map_closures::ClosureCandidate> retrieveCandidates(
      int query_id, const std::vector<Eigen::Vector3d>& query_points_mc) = 0;

  virtual void applyAcceptedClosure(const map_closures::ClosureCandidate& c,
                                    const Eigen::Matrix4d& query_odom_base) = 0;

 protected:
  // Accessors for Vegvisir internal state (avoids friend on derived classes)
  LocalMapGraph& localMapGraph();
  voxel_map::VoxelMap& voxelGrid();
  Eigen::Matrix4d& currentPose();
  Eigen::Matrix4d& tfMapOdom();
  double& distanceSinceQuery();
  kalman_filter::PoseKalmanFilter& poseFilter();
  std::mutex& closureMutex();
  map_closures::MapClosures *mapCloser();
  std::unordered_map<int, std::vector<Eigen::Vector3d>>& localMapPoints();
  const VegvisirConfig& config() const;
  const std::unordered_map<int, Eigen::Matrix4d>& referencePoses() const;
  void processLoopClosuresAsync(int query_id, std::vector<Eigen::Vector3d> query_points_mc,
                                std::vector<Eigen::Vector3d> query_points_icp,
                                Eigen::Matrix4d query_odom_base);

 private:
  Vegvisir& vegvisir_;
};

}  // namespace vegvisir
