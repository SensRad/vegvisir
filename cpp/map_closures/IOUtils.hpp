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
template <class T> inline bool write_pod(std::ostream &os, const T &v) {
  static_assert(std::is_trivially_copyable<T>::value, "T must be POD-like");
  return bool(os.write(reinterpret_cast<const char *>(&v), sizeof(T)));
}
template <class T> inline bool read_pod(std::istream &is, T &v) {
  static_assert(std::is_trivially_copyable<T>::value, "T must be POD-like");
  return bool(is.read(reinterpret_cast<char *>(&v), sizeof(T)));
}

// cv::Mat helpers -----------------------------------------------------------
inline bool write_mat(std::ostream &os, const cv::Mat &m) {
  const int rows = m.rows, cols = m.cols, type = m.type();
  if (!write_pod(os, rows) || !write_pod(os, cols) || !write_pod(os, type))
    return false;

  const size_t bytes = static_cast<size_t>(rows) * cols * m.elemSize();
  if (bytes == 0)
    return os.good();

  if (!m.data)
    return false;

  if (!m.isContinuous()) {
    cv::Mat tmp = m.clone();
    return write_mat(os, tmp);
  }
  return bool(os.write(reinterpret_cast<const char *>(m.data), bytes));
}

inline bool read_mat(std::istream &is, cv::Mat &m) {
  int rows = 0, cols = 0, type = 0;
  if (!read_pod(is, rows) || !read_pod(is, cols) || !read_pod(is, type))
    return false;

  if (rows == 0 || cols == 0) {
    m = cv::Mat();
    return is.good();
  }

  m.create(rows, cols, type);
  const size_t bytes = static_cast<size_t>(rows) * cols * m.elemSize();
  return bool(is.read(reinterpret_cast<char *>(m.data), bytes));
}

// generic map<K,V> helpers --------------------------------------------------
template <class K, class Writer>
inline bool write_map(std::ostream &os,
                      const std::unordered_map<K, typename Writer::Value> &m,
                      const Writer &w) {
  size_t n = m.size();
  if (!write_pod(os, n))
    return false;
  for (const auto &[k, v] : m) {
    if (!write_pod(os, k))
      return false;
    if (!w(os, v))
      return false;
  }
  return true;
}

template <class K, class Value, class Reader>
inline bool read_map(std::istream &is, std::unordered_map<K, Value> &m,
                     const Reader &r) {
  size_t n = 0;
  if (!read_pod(is, n))
    return false;
  m.clear();
  m.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    K k{};
    if (!read_pod(is, k))
      return false;
    Value v;
    if (!r(is, v))
      return false;
    m.emplace(k, std::move(v));
  }
  return true;
}

// Config serializer -----------------------------------------------------------
struct ConfigIO {
  using Value = map_closures::Config;
  bool operator()(std::ostream &os, const Value &c) const {
    return write_pod(os, c.density_map_resolution) &&
           write_pod(os, c.density_threshold) &&
           write_pod(os, c.sift_match_ratio) &&
           write_pod(os, c.lbd_min_line_length) &&
           write_pod(os, c.lbd_match_ratio) &&
           write_pod(os, c.lbd_num_octaves) && write_pod(os, c.lbd_scale) &&
           write_pod(os, c.density_map_gamma) && write_pod(os, c.lbd_weight);
  }
  bool operator()(std::istream &is, Value &c) const {
    return read_pod(is, c.density_map_resolution) &&
           read_pod(is, c.density_threshold) &&
           read_pod(is, c.sift_match_ratio) &&
           read_pod(is, c.lbd_min_line_length) &&
           read_pod(is, c.lbd_match_ratio) && read_pod(is, c.lbd_num_octaves) &&
           read_pod(is, c.lbd_scale) && read_pod(is, c.density_map_gamma) &&
           read_pod(is, c.lbd_weight);
  }
};

// DensityMap serializer -------------------------------------------------------
struct DensityMapIO {
  using Value = map_closures::DensityMap;
  bool operator()(std::ostream &os, const Value &d) const {
    if (!write_pod(os, d.lower_bound.x()) || !write_pod(os, d.lower_bound.y()))
      return false;
    return write_pod(os, d.resolution) && write_mat(os, d.grid);
  }
  bool operator()(std::istream &is, Value &d) const {
    int lb_x = 0, lb_y = 0;
    double res = 0.0;
    cv::Mat grid;
    if (!read_pod(is, lb_x) || !read_pod(is, lb_y) || !read_pod(is, res) ||
        !read_mat(is, grid))
      return false;
    Eigen::Vector2i lb(lb_x, lb_y);
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
        if (!write_pod(os, m(i, j)))
          return false;
      }
    }
    return true;
  }
  bool operator()(std::istream &is, Value &m) const {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        if (!read_pod(is, m(i, j)))
          return false;
      }
    }
    return true;
  }
};

} // namespace io
