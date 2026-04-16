// Copyright (c) Sensrad 2026

#pragma once

#include <cmath>
#include <cstdint>

#include <future>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "GnssState.hpp"
#include "LocalMapGraph.hpp"
#include "VegvisirConfig.hpp"
#include "VegvisirIO.hpp"
#include "icp_svd/IcpSvd.hpp"
#include "kalman_filter/PoseKalmanFilter.hpp"
#include "map_closures/MapClosures.hpp"
#include "pgo/pose_graph_optimizer.hpp"
#include "voxel_map/VoxelMap.hpp"

namespace vegvisir {

enum class Mode : std::uint8_t {
  LOCALIZATION,  // Localization-only mode (query existing map)
  SLAM           // SLAM mode (build map and optimize pose graph)
};

// Forward declaration for backend base class
class VegvisirBackend;

class Vegvisir {
 public:
  Vegvisir(const std::string& map_database_path, Mode mode = Mode::LOCALIZATION,
           const VegvisirConfig& config = VegvisirConfig{});
  ~Vegvisir();
  Vegvisir(const Vegvisir&) = delete;
  Vegvisir& operator=(const Vegvisir&) = delete;
  Vegvisir(Vegvisir&&) = delete;
  Vegvisir& operator=(Vegvisir&&) = delete;

  void update(const std::vector<Eigen::Vector3d>& points, const Sophus::SE3d& absolute_pose);

  // Save database (SLAM mode only) - saves keyposes.tum, points.ply, closures,
  // and metadata
  bool saveDatabase();

  // Get/set map metadata
  const MapMetadata& getMapMetadata() const { return map_metadata_; }
  void setMapMetadata(const MapMetadata& metadata) { map_metadata_ = metadata; }

  /// Set the GNSS anchor transform (T_ENU_map) to be saved with the map.
  void setGnssAnchorTransform(const Eigen::Matrix4d& pose_enu_map) {
    map_metadata_.gnss_anchor_transform = pose_enu_map;
    map_metadata_.has_gnss_anchor = true;
  }

  /// Set the GNSS origin (WGS84 coordinates) used for ENU conversion.
  void setGnssOrigin(double lat0, double lon0, double alt0) {
    map_metadata_.gnss_origin.lat0 = lat0;
    map_metadata_.gnss_origin.lon0 = lon0;
    map_metadata_.gnss_origin.alt0 = alt0;
    map_metadata_.gnss_origin.valid = true;
  }

  /// Get the GNSS anchor transform from loaded metadata.
  Eigen::Matrix4d getGnssAnchorTransform() const {
    return map_metadata_.has_gnss_anchor ? map_metadata_.gnss_anchor_transform
                                         : Eigen::Matrix4d::Identity();
  }

  /// Check if GNSS anchor data is available
  bool hasGnssAnchor() const { return map_metadata_.has_gnss_anchor; }

  /// Get the GNSS origin from loaded metadata.
  GnssOrigin getGnssOrigin() const { return map_metadata_.gnss_origin; }

  // Access to local map graph for external use (e.g., visualization)
  const LocalMapGraph& getLocalMapGraph() const { return local_map_graph_; }

  // Access to pose estimation state
  Eigen::Matrix4d getMapToOdomTransform() const { return tf_map_odom_; }
  Eigen::Matrix<double, 6, 6> getCovariance() const { return pose_filter_.covariance(); }
  Eigen::Matrix4d getBaseInMapFrame() const;
  Sophus::SE3d getCurrentOdomBase() const { return current_odom_base_; }

  // Access to map closure data for visualization and localization
  const std::unordered_map<int, Eigen::Matrix4d>& getReferencePoses() const;
  const Eigen::Matrix4d& getReferencePose(int map_id) const;
  const map_closures::DensityMap& getDensityMap(int map_id) const;
  std::vector<int> getAvailableMapIds() const;
  const Eigen::Matrix4d& getGroundAlignment(int map_id) const;
  const std::vector<map_closures::ClosureCandidate>& getClosures() const;
  size_t getNumClosures() const;

  // Access to local map points for ICP
  const std::vector<Eigen::Vector3d>& getLocalMapPoints(int map_id) const;
  bool hasLocalMapPoints(int map_id) const;

  // Fine-grained per-frame pose graph optimization over the full trajectory.
  // Returns the optimised per-frame poses.
  std::vector<Eigen::Matrix4d> fineGrainedOptimization();

  // Fine-grained PGO AND update internal keyposes to match optimized poses.
  // This ensures keyposes stored in the database match the PGO-optimized
  // trajectory.
  std::vector<Eigen::Matrix4d> fineGrainedOptimizationAndUpdateKeyposes();

  // GNSS measurement methods for pose graph optimization
  void addGnssMeasurement(int pose_index, const Eigen::Vector3d& position_enu,
                          const Eigen::Matrix3d& information_matrix) {
    gnss_state_.addMeasurement(pose_index, position_enu, information_matrix);
  }
  void clearGnssMeasurements() { gnss_state_.clearMeasurements(); }
  size_t getNumGnssMeasurements() const { return gnss_state_.numMeasurements(); }

  // Full SE3 GNSS pose measurement methods for pose graph optimization
  void addGnssPoseMeasurement(int pose_index, const Eigen::Matrix4d& pose_enu,
                              const Eigen::Matrix<double, 6, 6>& information_matrix) {
    gnss_state_.addPoseMeasurement(pose_index, pose_enu, information_matrix);
  }
  void clearGnssPoseMeasurements() { gnss_state_.clearPoseMeasurements(); }
  size_t getNumGnssPoseMeasurements() const { return gnss_state_.numPoseMeasurements(); }

  // GNSS alignment transform methods
  void setInitialAlignmentEstimate(const Eigen::Matrix4d& pose_enu_map) {
    gnss_state_.initial_pose_enu_map = pose_enu_map;
    gnss_state_.has_initial_alignment = true;
  }
  Eigen::Matrix4d getOptimizedAlignmentTransform() const {
    return gnss_state_.optimized_pose_enu_map;
  }

  // Get current mode
  Mode getMode() const { return mode_; }

  // Runtime configuration
  VegvisirConfig config_;

  // Radius of local voxel map to keep around current pose. Derived from config
  double localMapRadiusM() const { return 1.2 * config_.splitting_distance_slam; }

  // Maximum number of submaps in the localization ring buffer. Derived from
  // config
  int maxLocalizationSubmaps() const {
    return static_cast<int>(
               std::ceil(localMapRadiusM() / config_.splitting_distance_localization)) +
           2;
  }

  static constexpr int QUERY_ID_LOCALIZATION = 100000;  // reuse constant ID in localization

  static void transformAndAppendPoints(const std::vector<Eigen::Vector3d>& in,
                                       const Eigen::Matrix4d& transform_matrix,
                                       std::vector<Eigen::Vector3d>& out);

 private:
  // Shared mapping state/resources (used by both backends)
  voxel_map::VoxelMap voxel_grid_;
  LocalMapGraph local_map_graph_;

  // Shared update state (backend sets these)
  Eigen::Matrix4d current_pose_ = Eigen::Matrix4d::Identity();
  double distance_since_query_ = 0.0;

  // Detected closures (shared)
  std::vector<map_closures::ClosureCandidate> closures_;

  // Local map points storage (map_id -> point cloud) (shared)
  std::unordered_map<int, std::vector<Eigen::Vector3d>> local_map_points_;

  // Pose estimation state/output
  kalman_filter::PoseKalmanFilter pose_filter_;
  Eigen::Matrix4d tf_map_odom_ = Eigen::Matrix4d::Identity();
  Sophus::SE3d current_odom_base_;  // Current base_link pose in odom frame

  bool has_previous_pose_ = false;

  // Mode selection & persistence
  Mode mode_;
  std::string map_database_path_;  // Path to map directory for saving
  MapMetadata map_metadata_;       // Map metadata for organized storage

  // Loop closure state
  std::unique_ptr<map_closures::MapClosures> map_closer_;
  bool loop_closure_enabled_ = false;

  // GNSS state (measurements + alignment)
  GnssState gnss_state_;

  // Save state tracking (prevents redundant saves in destructor)
  bool database_saved_ = false;

  // Async loop closure state
  std::future<void> closure_future_;
  std::mutex closure_mutex_;

  // Backend (mode-specific policy/state)
  std::unique_ptr<VegvisirBackend> backend_;

  // Shared algorithms: ICP refinement + overlap validation
  std::pair<bool, Eigen::Matrix4d> performICPRefinement(
      const std::vector<Eigen::Vector3d>& query_points,
      const std::vector<Eigen::Vector3d>& reference_points,
      const Eigen::Matrix4d& initial_pose) const;

  bool validateClosurePose(const std::vector<Eigen::Vector3d>& query_points,
                           const std::vector<Eigen::Vector3d>& reference_points,
                           const Eigen::Matrix4d& pose) const;

  // Shared closure processing: candidate gating + ICP refine + overlap
  // validate. Retrieval + application are delegated to backend.
  // query_odom_base: the odom pose at the time the query was built.
  void processLoopClosures(int query_id, const std::vector<Eigen::Vector3d>& query_points_mc,
                           const std::vector<Eigen::Vector3d>& query_points_icp,
                           const Eigen::Matrix4d& query_odom_base);

  // Async wrapper: launches processLoopClosures on a background thread.
  // Skips if a previous closure job is still running.
  void processLoopClosuresAsync(int query_id, std::vector<Eigen::Vector3d> query_points_mc,
                                std::vector<Eigen::Vector3d> query_points_icp,
                                Eigen::Matrix4d query_odom_base);

  friend class VegvisirBackend;
};

}  // namespace vegvisir
