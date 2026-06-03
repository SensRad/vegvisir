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

// Reframe a stored ground-pixel keypoint into another segment's ground frame.
// Stored keypoints use the OpenCV convention x = col = ground_Y / resolution,
// y = row = ground_X / resolution (the lower_bound offset added in extract()
// cancels the one subtracted during rasterization). `ground_transform` maps the
// source ground frame to the target ground frame (3D). Returns the new (x, y).
inline cv::Point2f reframeGroundPixel(float x, float y, const Eigen::Matrix4d& ground_transform,
                                      double resolution) {
  const double ground_x = static_cast<double>(y) * resolution;
  const double ground_y = static_cast<double>(x) * resolution;
  const double ground_x_new = ground_transform(0, 0) * ground_x +
                              ground_transform(0, 1) * ground_y + ground_transform(0, 3);
  const double ground_y_new = ground_transform(1, 0) * ground_x +
                              ground_transform(1, 1) * ground_y + ground_transform(1, 3);
  return {static_cast<float>(ground_y_new / resolution),
          static_cast<float>(ground_x_new / resolution)};
}

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

  // Merge the source map's features into the target map, reframing source
  // keypoint positions by `ground_transform` (source ground -> target ground)
  // at the given density-map resolution, then erasing the source. Descriptors
  // are carried over unchanged (they are position/orientation invariant).
  virtual void mergeInto(int target_id, int source_id, const Eigen::Matrix4d& ground_transform,
                         double resolution) = 0;

  // Copy every entry from `other` into this layer at key (id + id_offset),
  // verbatim (keypoints/lines + descriptors unchanged). Used to combine two maps
  // that stay segment-wise separate (each keeps its own local frame); only their
  // keyposes/points are repositioned by the caller.
  virtual void importFrom(const FeatureLayer& other, int id_offset) = 0;

  [[nodiscard]] virtual std::size_t featureCount(int map_id) const = 0;

  virtual bool save(std::ostream& os) const = 0;
  virtual bool load(std::istream& is) = 0;

  [[nodiscard]] virtual std::vector<int> storedIds() const = 0;

  [[nodiscard]] virtual float matchRatio() const = 0;
};

}  // namespace map_closures
