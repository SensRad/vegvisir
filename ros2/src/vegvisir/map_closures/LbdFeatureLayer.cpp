// Copyright (c) Sensrad 2025-2026

#include "LbdFeatureLayer.hpp"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "IOUtils.hpp"

namespace map_closures {

LbdFeatureLayer::LbdFeatureLayer(const LbdConfig &config, float match_ratio)
    : match_ratio_(match_ratio), config_(config),
      lsd_(cv::line_descriptor::LSDDetector::createLSDDetector()),
      lbd_(cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor()) {}

void LbdFeatureLayer::extract(int map_id, const cv::Mat &gray_image,
                              const Eigen::Vector2i &lower_bound) {
  Entry entry;

  if (gray_image.empty()) {
    database_.insert_or_assign(map_id, std::move(entry));
    return;
  }

  // Suppress noise for cleaner line detection in density maps
  cv::Mat smoothed;
  cv::GaussianBlur(gray_image, smoothed, cv::Size(3, 3), 0);

  // Detect lines using LSD on the smoothed image
  std::vector<cv::line_descriptor::KeyLine> detected_lines;
  lsd_->detect(smoothed, detected_lines, config_.scale, config_.num_octaves);

  // Filter by minimum length
  std::copy_if(detected_lines.begin(), detected_lines.end(),
               std::back_inserter(entry.lines), [&](const auto &line) {
                 return line.lineLength >= config_.min_line_length;
               });

  if (!entry.lines.empty()) {
    // Compute LBD descriptors on the smoothed image
    try {
      lbd_->compute(smoothed, entry.lines, entry.descriptors);
    } catch (const cv::Exception &) {
      entry.lines.clear();
      entry.descriptors = cv::Mat();
    }
  }

  // Adjust line coordinates to global map frame
  for (auto &line : entry.lines) {
    line.startPointX += static_cast<float>(lower_bound.y());
    line.startPointY += static_cast<float>(lower_bound.x());
    line.endPointX += static_cast<float>(lower_bound.y());
    line.endPointY += static_cast<float>(lower_bound.x());
  }

  database_.insert_or_assign(map_id, std::move(entry));
}

std::vector<Correspondence>
LbdFeatureLayer::matchAgainstAll(int query_id, float ratio_threshold) const {
  std::vector<Correspondence> result;

  auto query_it = database_.find(query_id);
  if (query_it == database_.end() || query_it->second.descriptors.rows == 0) {
    return result;
  }

  const auto &query_lines = query_it->second.lines;
  const auto &query_desc = query_it->second.descriptors;

  for (const auto &[ref_id, ref_entry] : database_) {
    if (ref_id == query_id || ref_entry.descriptors.rows == 0) {
      continue;
    }

    std::vector<std::vector<cv::DMatch>> knn;
    bf_.knnMatch(query_desc, ref_entry.descriptors, knn, 2);

    // Ratio test + one-to-one enforcement
    std::unordered_map<int, cv::DMatch> best_by_train;
    for (const auto &match_pair : knn) {
      if (match_pair.size() >= 2 &&
          match_pair[0].distance < ratio_threshold * match_pair[1].distance) {
        auto it = best_by_train.find(match_pair[0].trainIdx);
        if (it == best_by_train.end() ||
            match_pair[0].distance < it->second.distance) {
          best_by_train[match_pair[0].trainIdx] = match_pair[0];
        }
      }
    }

    // Convert matched lines to point correspondences via midpoints
    const auto &ref_lines = ref_entry.lines;
    for (const auto &[_, m] : best_by_train) {
      const auto &q_line = query_lines[m.queryIdx];
      const auto &r_line = ref_lines[m.trainIdx];

      // Midpoints as correspondences (row, col) = (y, x)
      Eigen::Vector2d q_mid((q_line.startPointY + q_line.endPointY) / 2.0,
                            (q_line.startPointX + q_line.endPointX) / 2.0);
      Eigen::Vector2d r_mid((r_line.startPointY + r_line.endPointY) / 2.0,
                            (r_line.startPointX + r_line.endPointX) / 2.0);
      result.push_back({ref_id, 1, PointPair(r_mid, q_mid)});
    }
  }

  return result;
}

void LbdFeatureLayer::erase(int map_id) { database_.erase(map_id); }

std::size_t LbdFeatureLayer::featureCount(int map_id) const {
  auto it = database_.find(map_id);
  return it != database_.end() ? it->second.lines.size() : 0;
}

std::vector<int> LbdFeatureLayer::storedIds() const {
  auto keys = std::views::keys(database_);
  return {keys.begin(), keys.end()};
}

bool LbdFeatureLayer::save(std::ostream &os) const {
  if (!io::write_pod(os, static_cast<int>(database_.size())))
    return false;

  for (const auto &[map_id, entry] : database_) {
    if (!io::write_pod(os, map_id))
      return false;

    // Only persist geometry needed for midpoint correspondences
    if (!io::write_pod(os, static_cast<int>(entry.lines.size())))
      return false;
    for (const auto &line : entry.lines) {
      if (!io::write_pod(os, line.startPointX) ||
          !io::write_pod(os, line.startPointY) ||
          !io::write_pod(os, line.endPointX) ||
          !io::write_pod(os, line.endPointY) ||
          !io::write_pod(os, line.lineLength))
        return false;
    }

    // Write descriptors
    if (!io::write_mat(os, entry.descriptors))
      return false;
  }
  return true;
}

bool LbdFeatureLayer::load(std::istream &is) {
  int num_maps = 0;
  if (!io::read_pod(is, num_maps))
    return false;

  database_.clear();
  for (int i = 0; i < num_maps; ++i) {
    int map_id = 0;
    if (!io::read_pod(is, map_id))
      return false;

    int num_lines = 0;
    if (!io::read_pod(is, num_lines))
      return false;

    std::vector<cv::line_descriptor::KeyLine> lines;
    lines.reserve(num_lines);
    for (int j = 0; j < num_lines; ++j) {
      cv::line_descriptor::KeyLine line;
      if (!io::read_pod(is, line.startPointX) ||
          !io::read_pod(is, line.startPointY) ||
          !io::read_pod(is, line.endPointX) ||
          !io::read_pod(is, line.endPointY) ||
          !io::read_pod(is, line.lineLength))
        return false;
      lines.push_back(line);
    }

    cv::Mat descriptors;
    if (!io::read_mat(is, descriptors))
      return false;

    database_.insert_or_assign(map_id,
                               Entry{std::move(lines), std::move(descriptors)});
  }
  return true;
}

} // namespace map_closures
