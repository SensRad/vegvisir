// Copyright (c) Sensrad 2026

#pragma once

#include <cstddef>

#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "AlignRansac2D.hpp"

namespace map_closures {

struct Correspondence {
  int ref_map_id{};
  int layer_index{};
  PointPair pair;
};

class FeatureLayer {
 public:
  FeatureLayer() = default;
  virtual ~FeatureLayer() = default;
  FeatureLayer(const FeatureLayer&) = default;
  FeatureLayer(FeatureLayer&&) = default;
  FeatureLayer& operator=(const FeatureLayer&) = default;
  FeatureLayer& operator=(FeatureLayer&&) = default;

  virtual void extract(int map_id, const cv::Mat& gray_image,
                       const Eigen::Vector2i& lower_bound) = 0;

  [[nodiscard]] virtual std::vector<Correspondence> matchAgainstAll(
      int query_id, float ratio_threshold) const = 0;

  virtual void erase(int map_id) = 0;

  [[nodiscard]] virtual std::size_t featureCount(int map_id) const = 0;

  virtual bool save(std::ostream& os) const = 0;
  virtual bool load(std::istream& is) = 0;

  [[nodiscard]] virtual std::vector<int> storedIds() const = 0;

  [[nodiscard]] virtual float matchRatio() const = 0;
};

}  // namespace map_closures
