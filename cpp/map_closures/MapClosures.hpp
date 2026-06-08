// Copyright (c) Sensrad 2026
// Copyright (c) 2024 Saurabh Gupta, Tiziano Guadagnino, Benedikt Mersch,
// Ignacio Vizzo, Cyrill Stachniss.
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

#include <algorithm>
#include <iterator>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "DensityMap.hpp"
#include "FeatureLayer.hpp"

namespace map_closures {

static constexpr int MIN_NUMBER_OF_MATCHES = 10;
static constexpr int LOCAL_MAPS_TO_SKIP = 3;
static constexpr double GROUND_ALIGNMENT_RESOLUTION = 1.0;

struct Config {
  float density_map_resolution = 0.5F;
  float density_threshold = 0.05F;
  float sift_match_ratio = 0.85F;

  // LBD (Line Band Descriptor) parameters
  float lbd_min_line_length = 15.0F;
  float lbd_match_ratio = 0.80F;
  int lbd_num_octaves = 2;
  int lbd_scale = 2;

  // Gamma correction for density map
  float density_map_gamma = 0.3F;

  // Inlier weighting: weighted_score = sift_inliers + lbd_weight * lbd_inliers
  float lbd_weight = 3.0F;

  // RANSAC 2D alignment
  float ransac_inlier_threshold_m = 1.0F;
};

struct ClosureCandidate {
  int source_id = -1;
  int target_id = -1;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  std::size_t number_of_inliers = 0;
  double weighted_score = 0.0;
  std::size_t sift_inliers = 0;
  std::size_t lbd_inliers = 0;
};

// Transient per-query artifacts captured during a localization query, before
// they are erased from the detector. Used by the optional query recorder to
// dump the query's ground plane and density map to disk for offline
// evaluation. The density map is the input from which SIFT/LBD features are
// deterministically extracted, so features are reproducible from it offline.
struct QueryArtifacts {
  bool valid = false;
  Eigen::Matrix4d ground_alignment = Eigen::Matrix4d::Identity();
  cv::Mat density_grid;  // cloned (DensityMap is move-only)
  Eigen::Vector2i density_lower_bound{0, 0};
  double density_resolution = 0.0;
};

class MapClosures {
 public:
  MapClosures() : MapClosures(Config{}) {}
  explicit MapClosures(const Config& config);
  ~MapClosures() = default;
  MapClosures(const MapClosures&) = delete;
  MapClosures& operator=(const MapClosures&) = delete;
  MapClosures(MapClosures&&) = default;
  MapClosures& operator=(MapClosures&&) = default;

  std::vector<ClosureCandidate> getTopKClosures(int query_id,
                                                const std::vector<Eigen::Vector3d>& local_map,
                                                int k);
  std::vector<ClosureCandidate> getClosures(int query_id,
                                            const std::vector<Eigen::Vector3d>& local_map) {
    return getTopKClosures(query_id, local_map, -1);
  }

  // Query-only methods (match against database without adding to it).
  // If out_artifacts is non-null, the query's ground plane and density map are
  // copied into it before the transient query data is erased.
  std::vector<ClosureCandidate> queryTopKClosures(int query_id,
                                                  const std::vector<Eigen::Vector3d>& local_map,
                                                  int k,
                                                  QueryArtifacts* out_artifacts = nullptr);
  std::vector<ClosureCandidate> queryClosures(int query_id,
                                              const std::vector<Eigen::Vector3d>& local_map) {
    return queryTopKClosures(query_id, local_map, -1);
  }

  // Match an ALREADY-STORED segment against the other stored segments (no
  // re-extraction). Used to align two maps that were imported into one detector.
  // ignore_skip disables the sequential LOCAL_MAPS_TO_SKIP filter, which is wrong
  // across two maps whose ids are merely offset (not a single trajectory).
  std::vector<ClosureCandidate> getStoredClosures(int query_id, int k, bool ignore_skip);

  const DensityMap& getDensityMapFromId(int map_id) const { return density_maps_.at(map_id); }

  std::vector<int> getAvailableMapIds() const {
    std::vector<int> ids;
    ids.reserve(density_maps_.size());
    std::transform(density_maps_.begin(), density_maps_.end(), std::back_inserter(ids),
                   [](const auto& pair) { return pair.first; });
    std::sort(ids.begin(), ids.end());
    return ids;
  }

  const Eigen::Matrix4d& getGroundAlignment(int map_id) const {
    return ground_alignments_.at(map_id);
  }

  const std::unordered_map<int, Eigen::Matrix4d>& getReferencePoses() const {
    return reference_poses_;
  }

  const Eigen::Matrix4d& getReferencePose(int map_id) const { return reference_poses_.at(map_id); }

  void setReferencePose(int map_id, const Eigen::Matrix4d& pose) {
    reference_poses_[map_id] = pose;
  }

  void setGroundAlignment(int map_id, const Eigen::Matrix4d& ground_alignment) {
    ground_alignments_[map_id] = ground_alignment;
  }

  // Per-feature-layer feature counts for a map (e.g. {sift, lbd}).
  std::vector<std::size_t> getFeatureCounts(int map_id) const {
    std::vector<std::size_t> counts;
    counts.reserve(feature_layers_.size());
    for (const auto& layer : feature_layers_) {
      counts.push_back(layer->featureCount(map_id));
    }
    return counts;
  }

  // Merge the source map's features into the target map instead of re-extracting.
  // Source keypoints are reframed by relative_pose (source-local -> target-local)
  // composed with the stored ground alignments; the target's density map and
  // ground alignment are kept, and the source map is removed.
  void mergeMaps(int target_id, int source_id, const Eigen::Matrix4d& relative_pose);

  // Copy all segments (features, density maps, ground alignments) from another map
  // into this one, with their ids shifted by id_offset. Used to combine two maps
  // into one: segments stay separate and keep their own local frames (the caller
  // repositions their keyposes/points). ids must not collide with existing ones.
  void importSegments(const MapClosures& other, int id_offset);

  const std::unordered_map<int, uint64_t>& getReferenceTimestampsNs() const {
    return reference_timestamps_ns_;
  }

  const std::vector<Eigen::Vector3d>& getLocalMapPoints(int map_id) const {
    return local_map_points_.at(map_id);
  }

  bool hasLocalMapPoints(int map_id) const { return local_map_points_.contains(map_id); }

  // Persistence methods
  bool save(const std::string& file_path) const;
  bool load(const std::string& file_path);

  bool loadReferencePoses(const std::string& file_path, const std::vector<int>& keypose_ids);
  bool loadLocalMapPoints(const std::string& file_path);

 protected:
  static bool compareByWeightedScore(const ClosureCandidate& a, const ClosureCandidate& b);
  static bool isFarEnough(int ref_id, int query_id);

  void match(int id, const std::vector<Eigen::Vector3d>& local_map,
             std::vector<Correspondence>& out_correspondences);

  ClosureCandidate validateClosureWithMatches(int reference_id, int query_id,
                                              const std::vector<Correspondence>& matches) const;

  Config config_;
  std::vector<std::unique_ptr<FeatureLayer>> feature_layers_;
  std::unordered_map<int, DensityMap> density_maps_;
  std::unordered_map<int, Eigen::Matrix4d> ground_alignments_;
  std::unordered_map<int, Eigen::Matrix4d> reference_poses_;
  std::unordered_map<int, uint64_t> reference_timestamps_ns_;
  std::unordered_map<int, std::vector<Eigen::Vector3d>> local_map_points_;
};
}  // namespace map_closures
