// Copyright (c) Sensrad 2025-2026

#pragma once
#include <Eigen/Core>
#include <fstream>
#include <opencv2/core.hpp>
#include <type_traits>
#include <unordered_map>

#include "DensityMap.hpp"
#include "MapClosures.hpp"

namespace io {

// POD helpers ---------------------------------------------------------------
template <class T> inline bool writePod(std::ostream &os, const T &v) {
  static_assert(std::is_trivially_copyable_v<T>, "T must be POD-like");
  return bool(os.write(reinterpret_cast<const char *>(&v), sizeof(T)));
}
template <class T> inline bool readPod(std::istream &is, T &v) {
  static_assert(std::is_trivially_copyable_v<T>, "T must be POD-like");
  return bool(is.read(reinterpret_cast<char *>(&v), sizeof(T)));
}

// cv::Mat helpers -----------------------------------------------------------
// NOLINTNEXTLINE(misc-no-recursion)
inline bool writeMat(std::ostream &os, const cv::Mat &m) {
  const int rows = m.rows;
  const int cols = m.cols;
  const int type = m.type();
  if (!writePod(os, rows) || !writePod(os, cols) || !writePod(os, type)) {
    return false;
  }

  const size_t bytes = static_cast<size_t>(rows) * cols * m.elemSize();
  if (bytes == 0) {
    return os.good();
  }

  if (m.data == nullptr) {
    return false;
  }

  if (!m.isContinuous()) {
    const cv::Mat tmp = m.clone();
    return writeMat(os, tmp);
  }
  return bool(os.write(reinterpret_cast<const char *>(m.data),
                       static_cast<std::streamsize>(bytes)));
}

inline bool readMat(std::istream &is, cv::Mat &m) {
  int rows = 0;
  int cols = 0;
  int type = 0;
  if (!readPod(is, rows) || !readPod(is, cols) || !readPod(is, type)) {
    return false;
  }

  if (rows == 0 || cols == 0) {
    m = cv::Mat();
    return is.good();
  }

  m.create(rows, cols, type);
  const size_t bytes = static_cast<size_t>(rows) * cols * m.elemSize();
  return bool(is.read(reinterpret_cast<char *>(m.data),
                      static_cast<std::streamsize>(bytes)));
}

// generic map<K,V> helpers --------------------------------------------------
template <class K, class Writer>
inline bool writeMap(std::ostream &os,
                     const std::unordered_map<K, typename Writer::Value> &m,
                     const Writer &w) {
  const size_t n = m.size();
  if (!writePod(os, n)) {
    return false;
  }
  for (const auto &[k, v] : m) {
    if (!writePod(os, k)) {
      return false;
    }
    if (!w(os, v)) {
      return false;
    }
  }
  return true;
}

template <class K, class Value, class Reader>
inline bool readMap(std::istream &is, std::unordered_map<K, Value> &m,
                    const Reader &r) {
  size_t n = 0;
  if (!readPod(is, n)) {
    return false;
  }
  m.clear();
  m.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    K k{};
    if (!readPod(is, k)) {
      return false;
    }
    Value v;
    if (!r(is, v)) {
      return false;
    }
    m.emplace(k, std::move(v));
  }
  return true;
}

// Config serializer -----------------------------------------------------------
struct ConfigIO {
  using Value = map_closures::Config;
  bool operator()(std::ostream &os, const Value &c) const {
    return writePod(os, c.density_map_resolution) &&
           writePod(os, c.density_threshold) &&
           writePod(os, c.sift_match_ratio) &&
           writePod(os, c.lbd_min_line_length) &&
           writePod(os, c.lbd_match_ratio) &&
           writePod(os, c.lbd_num_octaves) && writePod(os, c.lbd_scale) &&
           writePod(os, c.density_map_gamma) && writePod(os, c.lbd_weight);
  }
  bool operator()(std::istream &is, Value &c) const {
    return readPod(is, c.density_map_resolution) &&
           readPod(is, c.density_threshold) &&
           readPod(is, c.sift_match_ratio) &&
           readPod(is, c.lbd_min_line_length) &&
           readPod(is, c.lbd_match_ratio) && readPod(is, c.lbd_num_octaves) &&
           readPod(is, c.lbd_scale) && readPod(is, c.density_map_gamma) &&
           readPod(is, c.lbd_weight);
  }
};

// DensityMap serializer -------------------------------------------------------
struct DensityMapIO {
  using Value = map_closures::DensityMap;
  bool operator()(std::ostream &os, const Value &d) const {
    if (!writePod(os, d.lower_bound.x()) || !writePod(os, d.lower_bound.y())) {
      return false;
    }
    return writePod(os, d.resolution) && writeMat(os, d.grid);
  }
  bool operator()(std::istream &is, Value &d) const {
    int lb_x = 0;
    int lb_y = 0;
    double res = 0.0;
    cv::Mat grid;
    if (!readPod(is, lb_x) || !readPod(is, lb_y) || !readPod(is, res) ||
        !readMat(is, grid)) {
      return false;
    }
    const Eigen::Vector2i lb(lb_x, lb_y);
    Value out(grid.rows, grid.cols, res, lb);
    out.grid = std::move(grid);
    d = std::move(out);
    return true;
  }
};

// Eigen::Matrix4d serializer --------------------------------------------------
struct Mat4IO {
  using Value = Eigen::Matrix4d;
  bool operator()(std::ostream &os, const Value &m) const {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        if (!writePod(os, m(i, j))) {
          return false;
        }
      }
    }
    return true;
  }
  bool operator()(std::istream &is, Value &m) const {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        if (!readPod(is, m(i, j))) {
          return false;
        }
      }
    }
    return true;
  }
};

} // namespace io
