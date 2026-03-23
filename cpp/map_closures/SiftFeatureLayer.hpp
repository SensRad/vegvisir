// Copyright (c) Sensrad 2025-2026

#pragma once

#include <ranges>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "FeatureLayer.hpp"

namespace map_closures {

class SiftFeatureLayer : public FeatureLayer {
 public:
  explicit SiftFeatureLayer(float match_ratio = 0.85F);

  void extract(int map_id, const cv::Mat& gray_image, const Eigen::Vector2i& lower_bound) override;

  std::vector<Correspondence> matchAgainstAll(int query_id, float ratio_threshold) const override;

  void erase(int map_id) override;

  std::size_t featureCount(int map_id) const override;

  bool save(std::ostream& os) const override;
  bool load(std::istream& is) override;

  std::vector<int> storedIds() const override;

  float matchRatio() const override { return match_ratio_; }

 private:
  // SIFT parameters
  static constexpr int NFEATURES = 0;  // 0 = unlimited
  static constexpr int NOCTAVE_LAYERS = 6;
  static constexpr double CONTRAST_THRESHOLD = 0.02;
  static constexpr double EDGE_THRESHOLD = 20.0;
  static constexpr double SIGMA = 1.6;

  struct Entry {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;  // rows x 128, CV_32F
  };

  float match_ratio_;
  cv::Ptr<cv::SIFT> sift_;
  cv::BFMatcher bf_{cv::NORM_L2};
  std::unordered_map<int, Entry> database_;
};

}  // namespace map_closures
