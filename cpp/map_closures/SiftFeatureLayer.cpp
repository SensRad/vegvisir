// Copyright (c) Sensrad 2025-2026

#include "SiftFeatureLayer.hpp"

#include "IOUtils.hpp"

namespace map_closures {

SiftFeatureLayer::SiftFeatureLayer(float match_ratio)
    : match_ratio_(match_ratio),
      sift_(
          cv::SIFT::create(NFEATURES, NOCTAVE_LAYERS, CONTRAST_THRESHOLD, EDGE_THRESHOLD, SIGMA)) {}

void SiftFeatureLayer::extract(int map_id, const cv::Mat& gray_image,
                               const Eigen::Vector2i& lower_bound) {
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  sift_->detectAndCompute(gray_image, cv::noArray(), keypoints, descriptors);

  // Adjust keypoint coordinates to global map frame
  for (auto& kp : keypoints) {
    kp.pt.x += static_cast<float>(lower_bound.y());
    kp.pt.y += static_cast<float>(lower_bound.x());
  }

  database_.insert_or_assign(map_id, Entry{std::move(keypoints), std::move(descriptors)});
}

std::vector<Correspondence> SiftFeatureLayer::matchAgainstAll(int query_id,
                                                              float ratio_threshold) const {
  std::vector<Correspondence> result;

  auto query_it = database_.find(query_id);
  if (query_it == database_.end() || query_it->second.descriptors.rows == 0) {
    return result;
  }

  const auto& query_kps = query_it->second.keypoints;
  const auto& query_desc = query_it->second.descriptors;

  for (const auto& [ref_id, ref_entry] : database_) {
    if (ref_id == query_id || ref_entry.descriptors.empty()) {
      continue;
    }

    std::vector<std::vector<cv::DMatch>> knn;
    bf_.knnMatch(query_desc, ref_entry.descriptors, knn, 2);

    // Ratio test + one-to-one enforcement
    std::unordered_map<int, cv::DMatch> best_by_train;
    for (const auto& match_pair : knn) {
      if (match_pair.size() >= 2 &&
          match_pair[0].distance < ratio_threshold * match_pair[1].distance) {
        auto it = best_by_train.find(match_pair[0].trainIdx);
        if (it == best_by_train.end() || match_pair[0].distance < it->second.distance) {
          best_by_train[match_pair[0].trainIdx] = match_pair[0];
        }
      }
    }

    // Convert to Correspondence (PointPair uses row,col = y,x)
    const auto& ref_kps = ref_entry.keypoints;
    for (const auto& [_, m] : best_by_train) {
      Eigen::Vector2d query_pt(query_kps[m.queryIdx].pt.y, query_kps[m.queryIdx].pt.x);
      Eigen::Vector2d ref_pt(ref_kps[m.trainIdx].pt.y, ref_kps[m.trainIdx].pt.x);
      result.push_back({ref_id, 0, PointPair(ref_pt, query_pt)});
    }
  }

  return result;
}

void SiftFeatureLayer::erase(int map_id) {
  database_.erase(map_id);
}

std::size_t SiftFeatureLayer::featureCount(int map_id) const {
  auto it = database_.find(map_id);
  return it != database_.end() ? it->second.keypoints.size() : 0;
}

std::vector<int> SiftFeatureLayer::storedIds() const {
  auto keys = std::views::keys(database_);
  return {keys.begin(), keys.end()};
}

bool SiftFeatureLayer::save(std::ostream& os) const {
  if (!io::write_pod(os, static_cast<int>(database_.size())))
    return false;

  for (const auto& [map_id, entry] : database_) {
    if (!io::write_pod(os, map_id))
      return false;

    // Write keypoints
    if (!io::write_pod(os, static_cast<int>(entry.keypoints.size())))
      return false;
    for (const auto& kp : entry.keypoints) {
      if (!io::write_pod(os, kp.pt.x) || !io::write_pod(os, kp.pt.y) ||
          !io::write_pod(os, kp.size) || !io::write_pod(os, kp.angle) ||
          !io::write_pod(os, kp.response))
        return false;
    }

    // Write descriptors
    if (!io::write_mat(os, entry.descriptors))
      return false;
  }
  return true;
}

bool SiftFeatureLayer::load(std::istream& is) {
  int num_maps = 0;
  if (!io::read_pod(is, num_maps))
    return false;

  database_.clear();
  for (int i = 0; i < num_maps; ++i) {
    int map_id = 0;
    if (!io::read_pod(is, map_id))
      return false;

    int num_kps = 0;
    if (!io::read_pod(is, num_kps))
      return false;

    std::vector<cv::KeyPoint> kps;
    kps.reserve(num_kps);
    for (int j = 0; j < num_kps; ++j) {
      cv::KeyPoint kp;
      if (!io::read_pod(is, kp.pt.x) || !io::read_pod(is, kp.pt.y) || !io::read_pod(is, kp.size) ||
          !io::read_pod(is, kp.angle) || !io::read_pod(is, kp.response))
        return false;
      kps.push_back(kp);
    }

    cv::Mat descriptors;
    if (!io::read_mat(is, descriptors))
      return false;

    database_.insert_or_assign(map_id, Entry{std::move(kps), std::move(descriptors)});
  }
  return true;
}

}  // namespace map_closures
