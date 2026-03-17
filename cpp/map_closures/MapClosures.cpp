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

#include "MapClosures.hpp"

#include <cmath>

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "AlignRansac2D.hpp"
#include "DensityMap.hpp"
#include "GroundAlign.hpp"
#include "IOUtils.hpp"
#include "LbdFeatureLayer.hpp"
#include "SiftFeatureLayer.hpp"

namespace {

struct FileHeader {
  std::array<char, 12> magic = {'M', 'A', 'P', 'C', 'L', 'O', 'S', 'U', 'R', 'E', 'S'};
  uint32_t version = 6;  // v6: FeatureLayer abstraction
};

}  // namespace

namespace map_closures {

MapClosures::MapClosures(const Config& config) : config_(config) {
  feature_layers_.push_back(std::make_unique<SiftFeatureLayer>(config_.sift_match_ratio));
  const LbdConfig lbd_cfg{config_.lbd_min_line_length, config_.lbd_num_octaves, config_.lbd_scale};
  feature_layers_.push_back(std::make_unique<LbdFeatureLayer>(lbd_cfg, config_.lbd_match_ratio));
}

bool MapClosures::compareByWeightedScore(const ClosureCandidate& a, const ClosureCandidate& b) {
  return a.weighted_score > b.weighted_score;
}

bool MapClosures::isFarEnough(int ref_id, int query_id) {
  return std::abs(query_id - ref_id) > LOCAL_MAPS_TO_SKIP;
}

void MapClosures::match(int id, const std::vector<Eigen::Vector3d>& local_map,
                        std::vector<Correspondence>& out_correspondences) {
  // Ground alignment and density map
  const Eigen::Matrix4d ground_transform =
      alignToLocalGround(local_map, GROUND_ALIGNMENT_RESOLUTION);
  DensityMap density_map = generateDensityMap(
      local_map, ground_transform, config_.density_map_resolution, config_.density_threshold);
  applyGammaCorrection(density_map, config_.density_map_gamma);

  // Extract features from all layers
  for (auto& layer : feature_layers_) {
    layer->extract(id, density_map.grid, density_map.lower_bound);
    auto correspondences = layer->matchAgainstAll(id, layer->matchRatio());
    out_correspondences.insert(out_correspondences.end(),
                               std::make_move_iterator(correspondences.begin()),
                               std::make_move_iterator(correspondences.end()));
  }

  // Store density map and ground alignment
  density_maps_.emplace(id, std::move(density_map));
  ground_alignments_.emplace(id, ground_transform);
}

ClosureCandidate MapClosures::validateClosureWithMatches(
    int reference_id, int query_id, const std::vector<Correspondence>& matches) const {
  // Filter correspondences for this reference map, track layer indices
  std::vector<PointPair> keypoint_pairs;
  std::vector<int> layer_indices;
  for (const auto& c : matches) {
    if (c.ref_map_id == reference_id) {
      keypoint_pairs.push_back(c.pair);
      layer_indices.push_back(c.layer_index);
    }
  }

  ClosureCandidate closure;
  if (keypoint_pairs.size() <= static_cast<size_t>(MIN_NUMBER_OF_MATCHES)) {
    return closure;
  }

  // RANSAC on combined correspondences
  auto [pose2d, point_inliers] = ransacAlignment2D(keypoint_pairs);
  if (point_inliers < 2) {
    return closure;
  }

  // Count RANSAC inliers per layer (threshold matches AlignRansac2D)
  std::size_t sift_inliers = 0;
  std::size_t lbd_inliers = 0;
  for (std::size_t i = 0; i < keypoint_pairs.size(); ++i) {
    if ((pose2d * keypoint_pairs[i].ref - keypoint_pairs[i].query).norm() <
        RANSAC_INLIER_THRESHOLD) {
      if (layer_indices[i] == 0) {
        ++sift_inliers;
      } else {
        ++lbd_inliers;
      }
    }
  }

  closure.source_id = reference_id;
  closure.target_id = query_id;
  closure.pose.block<2, 2>(0, 0) = pose2d.linear();
  closure.pose.block<2, 1>(0, 3) = pose2d.translation() * config_.density_map_resolution;
  closure.pose = ground_alignments_.at(query_id).inverse() * closure.pose *
                 ground_alignments_.at(reference_id);
  closure.number_of_inliers = point_inliers;
  closure.sift_inliers = sift_inliers;
  closure.lbd_inliers = lbd_inliers;
  closure.weighted_score =
      static_cast<double>(sift_inliers) +
      static_cast<double>(config_.lbd_weight) * static_cast<double>(lbd_inliers);

  return closure;
}

std::vector<ClosureCandidate> MapClosures::getTopKClosures(
    int query_id, const std::vector<Eigen::Vector3d>& local_map, int k) {
  std::vector<Correspondence> correspondences;
  match(query_id, local_map, correspondences);

  // Collect reference IDs from all layers
  std::vector<int> ref_ids;
  if (!feature_layers_.empty()) {
    ref_ids = feature_layers_[0]->storedIds();
  }

  std::vector<ClosureCandidate> closures;
  closures.reserve(ref_ids.size());

  for (const int ref_id : ref_ids) {
    if (!isFarEnough(ref_id, query_id)) {
      continue;
    }

    auto closure = validateClosureWithMatches(ref_id, query_id, correspondences);

    if (closure.weighted_score > static_cast<double>(MIN_NUMBER_OF_MATCHES)) {
      closures.emplace_back(closure);
    }
  }
  closures.shrink_to_fit();

  if (k != -1) {
    const auto top_k = std::min(static_cast<std::size_t>(k), closures.size());
    std::partial_sort(closures.begin(), closures.begin() + static_cast<std::ptrdiff_t>(top_k),
                      closures.end(), compareByWeightedScore);
    closures.resize(top_k);
  }
  return closures;
}

std::vector<ClosureCandidate> MapClosures::queryTopKClosures(
    int query_id, const std::vector<Eigen::Vector3d>& local_map, int k) {
  auto closures = getTopKClosures(query_id, local_map, k);

  // Clean up temporary data (remove query from internal storage)
  density_maps_.erase(query_id);
  ground_alignments_.erase(query_id);
  for (auto& layer : feature_layers_) {
    layer->erase(query_id);
  }

  return closures;
}

bool MapClosures::save(const std::string& file_path) const {
  std::ofstream out(file_path, std::ios::binary);
  if (!out) {
    std::cerr << "save|ERROR: open " << file_path << "\n";
    return false;
  }

  // Header
  const FileHeader hdr;
  if (!(io::writePod(out, hdr))) {
    return false;
  }

  // Config
  const io::ConfigIO cfgio;
  if (!cfgio(out, config_)) {
    return false;
  }

  // Density maps
  if (!io::writeMap<int, io::DensityMapIO>(out, density_maps_, io::DensityMapIO{})) {
    return false;
  }

  // Ground alignments
  if (!io::writeMap<int, io::Mat4IO>(out, ground_alignments_, io::Mat4IO{})) {
    return false;
  }

  // Feature layers
  for (const auto& layer : feature_layers_) {
    if (!layer->save(out)) {
      return false;
    }
  }

  if (!out.good()) {
    return false;
  }
  out.close();
  return true;
}

bool MapClosures::load(const std::string& file_path) {
  std::ifstream in(file_path, std::ios::binary);
  if (!in) {
    std::cerr << "load|ERROR: open " << file_path << "\n";
    return false;
  }

  // Header
  FileHeader hdr{};
  if (!io::readPod(in, hdr)) {
    return false;
  }
  if (std::string(hdr.magic.begin(), hdr.magic.begin() + 11) != "MAPCLOSURES" || hdr.version != 6) {
    std::cerr << "load|ERROR: bad header (version=" << hdr.version << ")\n";
    return false;
  }

  // Config
  const io::ConfigIO cfgio;
  if (!cfgio(in, config_)) {
    return false;
  }

  // Density maps
  if (!io::readMap<int, DensityMap, io::DensityMapIO>(in, density_maps_, io::DensityMapIO{})) {
    return false;
  }

  // Ground alignments
  if (!io::readMap<int, Eigen::Matrix4d, io::Mat4IO>(in, ground_alignments_, io::Mat4IO{})) {
    return false;
  }

  // Feature layers
  for (auto& layer : feature_layers_) {
    if (!layer->load(in)) {
      return false;
    }
  }

  in.close();
  return true;
}

bool MapClosures::loadReferencePoses(const std::string& file_path) {
  std::ifstream in(file_path, std::ios::binary);
  if (!in) {
    std::cerr << "loadReferencePoses|ERROR: open " << file_path << "\n";
    return false;
  }

  size_t num_poses = 0;
  if (!io::readPod(in, num_poses)) {
    std::cerr << "loadReferencePoses|ERROR: read num_poses\n";
    return false;
  }

  reference_poses_.clear();
  reference_poses_.reserve(num_poses);

  for (size_t i = 0; i < num_poses; ++i) {
    int map_id = 0;
    if (!io::readPod(in, map_id)) {
      std::cerr << "loadReferencePoses|ERROR: read map_id\n";
      return false;
    }

    Eigen::Matrix4d pose;
    if (!io::Mat4IO{}(in, pose)) {
      std::cerr << "loadReferencePoses|ERROR: read pose\n";
      return false;
    }

    reference_poses_[map_id] = pose;
  }

  std::cout << "Loaded " << reference_poses_.size() << " reference poses from " << file_path
            << '\n';
  return true;
}

bool MapClosures::loadLocalMapPoints(const std::string& file_path) {
  std::ifstream in(file_path, std::ios::binary);
  if (!in) {
    std::cerr << "loadLocalMapPoints|ERROR: open " << file_path << "\n";
    return false;
  }

  size_t num_maps = 0;
  if (!io::readPod(in, num_maps)) {
    std::cerr << "loadLocalMapPoints|ERROR: read num_maps\n";
    return false;
  }

  local_map_points_.clear();

  for (size_t i = 0; i < num_maps; ++i) {
    int map_id = 0;
    if (!io::readPod(in, map_id)) {
      std::cerr << "loadLocalMapPoints|ERROR: read map_id\n";
      return false;
    }

    size_t num_points = 0;
    if (!io::readPod(in, num_points)) {
      std::cerr << "loadLocalMapPoints|ERROR: read num_points\n";
      return false;
    }

    std::vector<Eigen::Vector3d> points;
    points.reserve(num_points);

    for (size_t j = 0; j < num_points; ++j) {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (!io::readPod(in, x) || !io::readPod(in, y) || !io::readPod(in, z)) {
        std::cerr << "loadLocalMapPoints|ERROR: read point coordinates\n";
        return false;
      }
      points.emplace_back(x, y, z);
    }

    local_map_points_[map_id] = std::move(points);
  }

  std::cout << "Loaded " << local_map_points_.size() << " local map point clouds from " << file_path
            << '\n';
  return true;
}

}  // namespace map_closures
