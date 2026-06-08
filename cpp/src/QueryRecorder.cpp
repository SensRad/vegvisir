// Copyright (c) Sensrad 2026

#include "QueryRecorder.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

#include "map_closures/IOUtils.hpp"

namespace fs = std::filesystem;

namespace vegvisir {

namespace {

// Write a point cloud as a binary little-endian PLY (x, y, z as double),
// following the convention used by saveLocalMapPointsPly in VegvisirIO.
bool savePointCloudPly(const fs::path& file_path, const std::vector<Eigen::Vector3d>& points) {
  std::ofstream file(file_path, std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "QueryRecorder: could not open " << file_path << " for writing\n";
    return false;
  }

  std::ostringstream header;
  header << "ply\n";
  header << "format binary_little_endian 1.0\n";
  header << "element vertex " << points.size() << "\n";
  header << "property double x\n";
  header << "property double y\n";
  header << "property double z\n";
  header << "end_header\n";
  const std::string header_str = header.str();
  file.write(header_str.data(), static_cast<std::streamsize>(header_str.size()));

  for (const auto& p : points) {
    const double x = p.x();
    const double y = p.y();
    const double z = p.z();
    file.write(reinterpret_cast<const char*>(&x), sizeof(double));
    file.write(reinterpret_cast<const char*>(&y), sizeof(double));
    file.write(reinterpret_cast<const char*>(&z), sizeof(double));
  }
  return file.good();
}

// Serialize a 4x4 matrix as 16 whitespace-separated, row-major doubles.
std::string matToRowMajor(const Eigen::Matrix4d& m) {
  std::ostringstream os;
  os << std::setprecision(12);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      os << m(i, j);
      if (i != 3 || j != 3) {
        os << ' ';
      }
    }
  }
  return os.str();
}

// Human-readable 4x4 ground plane transform (row per line).
bool saveGroundPlane(const fs::path& file_path, const Eigen::Matrix4d& ground_alignment) {
  std::ofstream file(file_path);
  if (!file.is_open()) {
    return false;
  }
  file << std::setprecision(12);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      file << ground_alignment(i, j);
      file << (j == 3 ? '\n' : ' ');
    }
  }
  return file.good();
}

// Binary density map: cv::Mat grid + lower_bound (x, y) + resolution.
bool saveDensityMap(const fs::path& file_path, const map_closures::QueryArtifacts& artifacts) {
  std::ofstream file(file_path, std::ios::binary);
  if (!file.is_open()) {
    return false;
  }
  const int lb_x = artifacts.density_lower_bound.x();
  const int lb_y = artifacts.density_lower_bound.y();
  return io::writePod(file, lb_x) && io::writePod(file, lb_y) &&
         io::writePod(file, artifacts.density_resolution) &&
         io::writeMat(file, artifacts.density_grid);
}

}  // namespace

LocalizationQueryRecorder::LocalizationQueryRecorder(std::string output_dir)
    : output_dir_(std::move(output_dir)) {
  std::error_code ec;
  fs::create_directories(output_dir_, ec);
  if (ec) {
    std::cerr << "QueryRecorder: failed to create output dir " << output_dir_ << ": "
              << ec.message() << '\n';
  } else {
    std::cout << "QueryRecorder: saving localization queries to " << output_dir_ << '\n';
  }
}

void LocalizationQueryRecorder::record(uint64_t timestamp_ns, int query_id,
                                       const Eigen::Matrix4d& query_odom_base,
                                       const Eigen::Matrix4d& tf_map_odom, double voxel_size,
                                       const std::vector<Eigen::Vector3d>& query_points_mc,
                                       const std::vector<Eigen::Vector3d>& query_points_icp,
                                       const map_closures::QueryArtifacts& artifacts,
                                       const std::vector<QueryMatchRecord>& matches) {
  const uint64_t index = query_counter_++;

  std::ostringstream dir_name;
  dir_name << "query_" << std::setw(6) << std::setfill('0') << index << '_' << timestamp_ns;
  const fs::path query_dir = fs::path(output_dir_) / dir_name.str();

  std::error_code ec;
  fs::create_directories(query_dir, ec);
  if (ec) {
    std::cerr << "QueryRecorder: failed to create " << query_dir << ": " << ec.message() << '\n';
    return;
  }

  savePointCloudPly(query_dir / "query_cloud.ply", query_points_mc);
  savePointCloudPly(query_dir / "query_icp.ply", query_points_icp);

  if (artifacts.valid) {
    saveGroundPlane(query_dir / "ground_plane.txt", artifacts.ground_alignment);
    saveDensityMap(query_dir / "density_map.bin", artifacts);
  }

  // Metadata (manual YAML, mirroring the style used in VegvisirIO::saveMetadata).
  std::ofstream meta(query_dir / "meta.yaml");
  if (!meta.is_open()) {
    std::cerr << "QueryRecorder: could not write meta.yaml in " << query_dir << '\n';
    return;
  }
  meta << "query_index: " << index << '\n';
  meta << "query_id: " << query_id << '\n';
  meta << "timestamp_ns: " << timestamp_ns << '\n';
  meta << "voxel_size: " << voxel_size << '\n';
  meta << "num_points_mc: " << query_points_mc.size() << '\n';
  meta << "num_points_icp: " << query_points_icp.size() << '\n';
  meta << "# row-major 4x4 transforms\n";
  meta << "tf_odom_base: [" << matToRowMajor(query_odom_base) << "]\n";
  meta << "tf_map_odom: [" << matToRowMajor(tf_map_odom) << "]\n";

  meta << "density_map_valid: " << (artifacts.valid ? "true" : "false") << '\n';
  if (artifacts.valid) {
    meta << "density_resolution: " << artifacts.density_resolution << '\n';
    meta << "density_lower_bound: [" << artifacts.density_lower_bound.x() << ", "
         << artifacts.density_lower_bound.y() << "]\n";
    meta << "density_rows: " << artifacts.density_grid.rows << '\n';
    meta << "density_cols: " << artifacts.density_grid.cols << '\n';
  }

  meta << "num_candidates: " << matches.size() << '\n';
  meta << "candidates:\n";
  for (const auto& m : matches) {
    meta << "  - source_id: " << m.source_id << '\n';
    meta << "    number_of_inliers: " << m.number_of_inliers << '\n';
    meta << "    sift_inliers: " << m.sift_inliers << '\n';
    meta << "    lbd_inliers: " << m.lbd_inliers << '\n';
    meta << "    weighted_score: " << m.weighted_score << '\n';
    meta << "    icp_converged: " << (m.icp_converged ? "true" : "false") << '\n';
    meta << "    accepted: " << (m.accepted ? "true" : "false") << '\n';
    meta << "    ransac_pose: [" << matToRowMajor(m.ransac_pose) << "]\n";
    meta << "    refined_pose: [" << matToRowMajor(m.refined_pose) << "]\n";
  }
}

}  // namespace vegvisir
