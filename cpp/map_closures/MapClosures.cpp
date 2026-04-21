// Copyright (c) Sensrad 2026
// Copyright (c) 2024 Saurabh Gupta, Tiziano Guadagnino, Benedikt Mersch,
// Ignacio Vizzo, Cyrill Stachniss.
// SPDX-License-Identifier: MIT

#include "MapClosures.hpp"

#include <cmath>

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
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

  // RANSAC on combined correspondences (threshold in pixels = meters /
  // resolution)
  const double inlier_threshold_px = static_cast<double>(config_.ransac_inlier_threshold_m) /
                                     static_cast<double>(config_.density_map_resolution);
  auto [pose2d, point_inliers] = ransacAlignment2D(keypoint_pairs, inlier_threshold_px);
  if (point_inliers < 2) {
    return closure;
  }

  // Count RANSAC inliers per layer (same pixel threshold as RANSAC)
  std::size_t sift_inliers = 0;
  std::size_t lbd_inliers = 0;
  for (std::size_t i = 0; i < keypoint_pairs.size(); ++i) {
    if ((pose2d * keypoint_pairs[i].ref - keypoint_pairs[i].query).norm() < inlier_threshold_px) {
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

bool MapClosures::loadReferencePoses(const std::string& file_path,
                                     const std::vector<int>& keypose_ids) {
  std::ifstream in(file_path);
  if (!in) {
    std::cerr << "loadReferencePoses|ERROR: open " << file_path << "\n";
    return false;
  }

  reference_poses_.clear();
  reference_timestamps_ns_.clear();

  // Standard TUM format: timestamp tx ty tz qx qy qz qw. Row k binds to
  // map_id keypose_ids[k] (carried in metadata.yaml).
  std::string line;
  size_t row = 0;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream ss(line);
    double ts = 0.0;
    double tx = 0.0, ty = 0.0, tz = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

    if (!(ss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
      std::cerr << "loadReferencePoses|ERROR: parse TUM line: " << line << "\n";
      return false;
    }

    if (row >= keypose_ids.size()) {
      std::cerr << "loadReferencePoses|ERROR: more TUM rows than keypose_ids entries ("
                << keypose_ids.size() << ") in metadata\n";
      return false;
    }

    Eigen::Quaterniond q(qw, qx, qy, qz);
    q.normalize();

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    pose.block<3, 1>(0, 3) = Eigen::Vector3d(tx, ty, tz);

    const int map_id = keypose_ids[row];
    reference_poses_[map_id] = pose;
    reference_timestamps_ns_[map_id] = static_cast<uint64_t>(std::llround(ts * 1e9));
    ++row;
  }

  if (row != keypose_ids.size()) {
    std::cerr << "loadReferencePoses|ERROR: keypose_ids length (" << keypose_ids.size()
              << ") does not match TUM row count (" << row << ")\n";
    reference_poses_.clear();
    reference_timestamps_ns_.clear();
    return false;
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

  local_map_points_.clear();

  // Parse PLY header
  std::string line;
  uint64_t num_vertices = 0;

  while (std::getline(in, line)) {
    if (line.rfind("element vertex ", 0) == 0) {
      num_vertices = std::stoull(line.substr(15));
    }
    if (line == "end_header") {
      break;
    }
  }

  // Read binary vertex data: 3 doubles + 1 int32 per vertex
  for (uint64_t i = 0; i < num_vertices; ++i) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int32_t map_id = 0;
    in.read(reinterpret_cast<char *>(&x), sizeof(double));
    in.read(reinterpret_cast<char *>(&y), sizeof(double));
    in.read(reinterpret_cast<char *>(&z), sizeof(double));
    in.read(reinterpret_cast<char *>(&map_id), sizeof(int32_t));

    if (!in) {
      std::cerr << "loadLocalMapPoints|ERROR: PLY unexpected end of file\n";
      return false;
    }
    local_map_points_[map_id].emplace_back(x, y, z);
  }

  // PLY stores points in map frame; transform back to local keypose frames
  if (!reference_poses_.empty()) {
    for (auto& [map_id, points] : local_map_points_) {
      auto pose_it = reference_poses_.find(map_id);
      if (pose_it == reference_poses_.end()) {
        continue;
      }
      const Eigen::Matrix3d r_inv = pose_it->second.block<3, 3>(0, 0).transpose();
      const Eigen::Vector3d t = pose_it->second.block<3, 1>(0, 3);
      for (auto& pt : points) {
        pt = r_inv * (pt - t);
      }
    }
  }

  size_t total_points = 0;
  for (const auto& [id, pts] : local_map_points_) {
    total_points += pts.size();
  }
  std::cout << "Loaded " << total_points << " points (" << local_map_points_.size()
            << " submaps) from " << file_path << '\n';
  return true;
}

}  // namespace map_closures
