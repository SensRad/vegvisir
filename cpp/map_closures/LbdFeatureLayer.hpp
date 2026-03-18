// Copyright (c) Sensrad 2025-2026

#pragma once

#include <ranges>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/line_descriptor.hpp>

#include "FeatureLayer.hpp"

namespace map_closures {

struct LbdConfig {
  float min_line_length = 15.0F;
  int num_octaves = 2;
  int scale = 2;
};

class LbdFeatureLayer : public FeatureLayer {
 public:
  explicit LbdFeatureLayer(const LbdConfig& config = {}, float match_ratio = 0.80F);

  void extract(int map_id, const cv::Mat& gray_image, const Eigen::Vector2i& lower_bound) override;

  std::vector<Correspondence> matchAgainstAll(int query_id, float ratio_threshold) const override;

  void erase(int map_id) override;

  std::size_t featureCount(int map_id) const override;

  bool save(std::ostream& os) const override;
  bool load(std::istream& is) override;

  std::vector<int> storedIds() const override;

  float matchRatio() const override { return match_ratio_; }

 private:
  struct Entry {
    std::vector<cv::line_descriptor::KeyLine> lines;
    cv::Mat descriptors;  // rows x 32, CV_8U (binary)
  };

  float match_ratio_;
  LbdConfig config_;
  cv::Ptr<cv::line_descriptor::LSDDetector> lsd_;
  cv::Ptr<cv::line_descriptor::BinaryDescriptor> lbd_;
  cv::BFMatcher bf_{cv::NORM_HAMMING};
  std::unordered_map<int, Entry> database_;
};

}  // namespace map_closures
