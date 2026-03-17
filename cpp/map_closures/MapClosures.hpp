// Copyright (c) Sensrad 2025-2026
//
// MIT License
//
// Copyright (c) 2024 Saurabh Gupta, Tiziano Guadagnino, Benedikt Mersch,
// Ignacio Vizzo, Cyrill Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

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
static constexpr double GROUND_ALIGNMENT_RESOLUTION = 5.0;

struct Config {
  float density_map_resolution = 0.5f;
  float density_threshold = 0.05f;
  float sift_match_ratio = 0.85f;

  // LBD (Line Band Descriptor) parameters
  float lbd_min_line_length = 15.0f;
  float lbd_match_ratio = 0.80f;
  int lbd_num_octaves = 2;
  int lbd_scale = 2;

  // Gamma correction for density map
  float density_map_gamma = 0.3f;

  // Inlier weighting: weighted_score = sift_inliers + lbd_weight * lbd_inliers
  float lbd_weight = 3.0f;
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

class MapClosures {
 public:
  MapClosures() : MapClosures(Config{}) {}
  explicit MapClosures(const Config& config);
  ~MapClosures() = default;

  std::vector<ClosureCandidate> GetTopKClosures(const int query_id,
                                                const std::vector<Eigen::Vector3d>& local_map,
                                                const int k);
  std::vector<ClosureCandidate> GetClosures(const int query_id,
                                            const std::vector<Eigen::Vector3d>& local_map) {
    return GetTopKClosures(query_id, local_map, -1);
  }

  // Query-only methods (match against database without adding to it)
  std::vector<ClosureCandidate> QueryTopKClosures(const int query_id,
                                                  const std::vector<Eigen::Vector3d>& local_map,
                                                  const int k);
  std::vector<ClosureCandidate> QueryClosures(const int query_id,
                                              const std::vector<Eigen::Vector3d>& local_map) {
    return QueryTopKClosures(query_id, local_map, -1);
  }

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

  const std::vector<Eigen::Vector3d>& getLocalMapPoints(int map_id) const {
    return local_map_points_.at(map_id);
  }

  bool hasLocalMapPoints(int map_id) const { return local_map_points_.contains(map_id); }

  // Persistence methods
  bool save(const std::string& file_path) const;
  bool load(const std::string& file_path);

  bool loadReferencePoses(const std::string& file_path);
  bool loadLocalMapPoints(const std::string& file_path);

 protected:
  static bool compareByWeightedScore(const ClosureCandidate& a, const ClosureCandidate& b);
  static bool isFarEnough(int ref_id, int query_id);

  void match(int id, const std::vector<Eigen::Vector3d>& local_map,
             std::vector<Correspondence>& out_correspondences);

  ClosureCandidate ValidateClosureWithMatches(int reference_id, int query_id,
                                              const std::vector<Correspondence>& matches) const;

  Config config_;
  std::vector<std::unique_ptr<FeatureLayer>> feature_layers_;
  std::unordered_map<int, DensityMap> density_maps_;
  std::unordered_map<int, Eigen::Matrix4d> ground_alignments_;
  std::unordered_map<int, Eigen::Matrix4d> reference_poses_;
  std::unordered_map<int, std::vector<Eigen::Vector3d>> local_map_points_;
};
}  // namespace map_closures
