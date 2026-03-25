// Copyright (c) Sensrad 2026

#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "LocalMapGraph.hpp"
#include "map_closures/MapClosures.hpp"

#include <sys/stat.h>

namespace vegvisir {

// GNSS origin for ENU coordinate system
struct GnssOrigin {
  double lat0 = 0.0;   // Latitude in degrees
  double lon0 = 0.0;   // Longitude in degrees
  double alt0 = 0.0;   // Altitude in meters
  bool valid = false;  // Whether origin is set
};

// Map metadata structure for organized map storage
struct MapMetadata {
  std::string name;      // Map name
  std::string location;  // Location name
  std::string notes;     // Additional notes

  // File names within the map directory
  struct Files {
    std::string database = "map_closures.db";
    std::string points = "points.ply";
    std::string poses = "keyposes.tum";
  } files;

  // GNSS anchoring data (for aligning map to global coordinates)
  Eigen::Matrix4d gnss_anchor_transform = Eigen::Matrix4d::Identity();
  bool has_gnss_anchor = false;
  GnssOrigin gnss_origin;
};

// Result of database loading operation
struct DatabaseLoadResult {
  bool success{};
  bool loop_closure_enabled{};
  std::string message;
};

// Save metadata to YAML file
// Returns true on success, false on failure
bool saveMetadata(const std::string& map_dir, const MapMetadata& metadata);

// Load metadata from YAML file
// Returns true on success, false on failure
bool loadMetadata(const std::string& map_dir, MapMetadata& metadata);

// Load complete database from map directory (map closures, poses, and points)
// map_dir: path to map directory containing metadata.yaml and map files
// require_exists: if true (LOCALIZATION mode), fails if database doesn't exist
//                 if false (SLAM mode), OK if database doesn't exist
DatabaseLoadResult loadDatabase(
    const std::string& map_dir, map_closures::MapClosures& map_closer,
    std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points, bool require_exists);

// Load local map points from binary file
// Returns true on success, false on failure
bool loadLocalMapPoints(const std::string& points_path,
                        std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points);

// Save poses in TUM format (map_id tx ty tz qx qy qz qw).
// Readable by evo, Open3D, and other standard SLAM tools.
// Returns true on success, false on failure
bool savePosesTum(const std::string& file_path, const LocalMapGraph& local_map_graph);

// Save local map point clouds in PLY format (binary little-endian).
// Each vertex stores x, y, z (double) and map_id (int32).
// The resulting file can be opened directly in Open3D, CloudCompare, etc.
// Returns true on success, false on failure
bool saveLocalMapPointsPly(
    const std::string& file_path, const LocalMapGraph& local_map_graph,
    const std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points);

// Save complete database to map directory (map closures, poses, points, and
// metadata) map_dir: path to map directory (will be created if it doesn't
// exist) metadata: map metadata to save Returns true on success, false on
// failure
bool saveDatabase(const std::string& map_dir, const MapMetadata& metadata,
                  map_closures::MapClosures& map_closer, const LocalMapGraph& local_map_graph,
                  const std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points);

// Rebuild LocalMapGraph from loaded reference poses and local map points
// This is useful for localization mode to have access to the map structure
void rebuildLocalMapGraph(
    LocalMapGraph& local_map_graph, const std::unordered_map<int, Eigen::Matrix4d>& reference_poses,
    const std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points);

// Extract map name from directory path
std::string extractMapName(const std::string& map_dir);

// Create default metadata for a map directory
MapMetadata createDefaultMetadata(const std::string& map_dir);

}  // namespace vegvisir
