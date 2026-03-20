// Copyright (c) Sensrad 2025-2026

#include "VegvisirIO.hpp"

#include <cmath>
#include <cstdint>

#include <filesystem>
#include <iomanip>
#include <regex>
#include <sstream>

namespace fs = std::filesystem;

namespace vegvisir {

std::string extractMapName(const std::string& map_dir) {
  fs::path p(map_dir);
  // Remove trailing slashes and get the last component
  while (p.filename().empty() && p.has_parent_path()) {
    p = p.parent_path();
  }
  return p.filename().string();
}

MapMetadata createDefaultMetadata(const std::string& map_dir) {
  MapMetadata metadata;
  metadata.name = extractMapName(map_dir);
  metadata.location = "";
  metadata.notes = "";
  // Use default file names from struct initialization
  return metadata;
}

bool saveMetadata(const std::string& map_dir, const MapMetadata& metadata) {
  const fs::path metadata_path = fs::path(map_dir) / "metadata.yaml";
  std::ofstream file(metadata_path);

  if (!file.is_open()) {
    std::cerr << "Could not open metadata file for writing: " << metadata_path << '\n';
    return false;
  }

  // Write YAML manually (simple format, no external dependency needed)
  file << "name: \"" << metadata.name << "\"\n";
  file << "location: \"" << metadata.location << "\"\n";
  file << "notes: \"" << metadata.notes << "\"\n";
  file << "files:\n";
  file << "  database: \"" << metadata.files.database << "\"\n";
  file << "  points: \"" << metadata.files.points << "\"\n";
  file << "  poses: \"" << metadata.files.poses << "\"\n";

  // Write GNSS anchor transform if available
  if (metadata.has_gnss_anchor) {
    file << "gnss_anchor_transform:\n";
    const auto& t = metadata.gnss_anchor_transform;
    for (int i = 0; i < 4; ++i) {
      file << "  - [" << t(i, 0) << ", " << t(i, 1) << ", " << t(i, 2) << ", " << t(i, 3) << "]\n";
    }

    if (metadata.gnss_origin.valid) {
      file << "gnss_origin:\n";
      file << std::fixed << std::setprecision(8);
      file << "  lat0: " << metadata.gnss_origin.lat0 << "\n";
      file << "  lon0: " << metadata.gnss_origin.lon0 << "\n";
      file << std::setprecision(4);
      file << "  alt0: " << metadata.gnss_origin.alt0 << "\n";
    }
  }

  file.close();
  std::cout << "Saved metadata to: " << metadata_path << '\n';
  return true;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
bool loadMetadata(const std::string& map_dir, MapMetadata& metadata) {
  const fs::path metadata_path = fs::path(map_dir) / "metadata.yaml";
  std::ifstream file(metadata_path);

  if (!file.is_open()) {
    std::cerr << "Could not open metadata file: " << metadata_path << '\n';
    return false;
  }

  // Simple YAML parser for our specific format
  std::string line;
  const std::regex key_value_regex(R"(^\s*(\w+):\s*\"?([^\"]*)\"?\s*$)");
  const std::regex nested_key_value_regex(R"(^\s{2}(\w+):\s*\"?([^\"]*)\"?\s*$)");
  const std::regex array_row_regex(R"(^\s+-\s*\[([^\]]+)\]\s*$)");

  enum class Section : std::uint8_t { NONE, FILES, GNSS_TRANSFORM, GNSS_ORIGIN };
  Section current_section = Section::NONE;
  int transform_row = 0;

  while (std::getline(file, line)) {
    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::smatch match;

    // Check for array rows (gnss_anchor_transform)
    if (current_section == Section::GNSS_TRANSFORM &&
        std::regex_match(line, match, array_row_regex)) {
      const std::string values_str = match[1].str();
      std::stringstream ss(values_str);
      std::string token;
      int col = 0;
      while (std::getline(ss, token, ',') && col < 4) {
        metadata.gnss_anchor_transform(transform_row, col) = std::stod(token);
        col++;
      }
      transform_row++;
      if (transform_row >= 4) {
        metadata.has_gnss_anchor = true;
        current_section = Section::NONE;
      }
      continue;
    }

    // Check for nested keys
    if (current_section == Section::FILES &&
        std::regex_match(line, match, nested_key_value_regex)) {
      const std::string key = match[1].str();
      const std::string value = match[2].str();

      if (key == "database") {
        metadata.files.database = value;
      } else if (key == "points") {
        metadata.files.points = value;
      } else if (key == "poses") {
        metadata.files.poses = value;
      }
    } else if (current_section == Section::GNSS_ORIGIN &&
               std::regex_match(line, match, nested_key_value_regex)) {
      const std::string key = match[1].str();
      const std::string value = match[2].str();

      if (key == "lat0") {
        metadata.gnss_origin.lat0 = std::stod(value);
      } else if (key == "lon0") {
        metadata.gnss_origin.lon0 = std::stod(value);
      } else if (key == "alt0") {
        metadata.gnss_origin.alt0 = std::stod(value);
        metadata.gnss_origin.valid = true;
      }
    }
    // Check for top-level keys
    else if (std::regex_match(line, match, key_value_regex)) {
      const std::string key = match[1].str();
      const std::string value = match[2].str();

      if (key == "name") {
        metadata.name = value;
        current_section = Section::NONE;
      } else if (key == "location") {
        metadata.location = value;
        current_section = Section::NONE;
      } else if (key == "notes") {
        metadata.notes = value;
        current_section = Section::NONE;
      } else if (key == "files") {
        current_section = Section::FILES;
      } else if (key == "gnss_anchor_transform") {
        current_section = Section::GNSS_TRANSFORM;
        transform_row = 0;
      } else if (key == "gnss_origin") {
        current_section = Section::GNSS_ORIGIN;
      }
    }
    // Check if we're exiting a section (non-indented, non-array line)
    else if (current_section != Section::NONE && !line.empty() && line[0] != ' ' &&
             line[0] != '-') {
      current_section = Section::NONE;
    }
  }

  file.close();
  std::cout << "Loaded metadata from: " << metadata_path << '\n';
  std::cout << "  Map name: " << metadata.name << '\n';
  std::cout << "  Location: " << metadata.location << '\n';
  if (metadata.has_gnss_anchor) {
    const double yaw =
        std::atan2(metadata.gnss_anchor_transform(1, 0), metadata.gnss_anchor_transform(0, 0));
    std::cout << "  GNSS anchor: yaw=" << (yaw * 180.0 / M_PI) << "°" << '\n';
  }
  return true;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
DatabaseLoadResult loadDatabase(
    const std::string& map_dir, map_closures::MapClosures& map_closer,
    std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points, bool require_exists) {
  DatabaseLoadResult result;
  result.success = true;
  result.loop_closure_enabled = false;

  // Check if map directory exists
  const bool dir_exists = fs::exists(map_dir) && fs::is_directory(map_dir);

  // Load or create default metadata
  MapMetadata metadata;
  if (dir_exists) {
    if (!loadMetadata(map_dir, metadata)) {
      // Use defaults if metadata file doesn't exist
      metadata = createDefaultMetadata(map_dir);
      std::cout << "Using default metadata for map directory" << '\n';
    }
  } else {
    metadata = createDefaultMetadata(map_dir);
  }

  // Construct file paths from metadata
  const fs::path db_path = fs::path(map_dir) / metadata.files.database;
  const fs::path poses_path = fs::path(map_dir) / metadata.files.poses;
  const fs::path points_path = fs::path(map_dir) / metadata.files.points;

  // Check if database file exists
  const bool database_exists = fs::exists(db_path);

  if (require_exists) {
    // LOCALIZATION mode: Database MUST exist
    if (!dir_exists || !database_exists) {
      result.success = false;
      result.loop_closure_enabled = false;
      result.message =
          "ERROR: Localization mode requires existing map database at: " + db_path.string() +
          ". Map directory or database file does not exist.";
      return result;
    }

    if (!map_closer.load(db_path.string())) {
      result.success = false;
      result.loop_closure_enabled = false;
      result.message = "ERROR: Failed to load map database from: " + db_path.string();
      return result;
    }

    result.message =
        "Localization mode: Successfully loaded map database from: " + db_path.string();
    result.loop_closure_enabled = true;

  } else {
    // SLAM mode: OK if database doesn't exist, we'll create a new one
    if (database_exists) {
      if (map_closer.load(db_path.string())) {
        result.message = "SLAM mode: Loaded existing map database from: " + db_path.string();
      } else {
        result.message =
            "Warning: Failed to load existing database, starting "
            "with empty map";
      }
    } else {
      result.message = "SLAM mode: Starting with new empty map at: " + map_dir;
    }
    result.loop_closure_enabled = true;
  }

  // Try to load reference poses (only if database was loaded)
  if (database_exists) {
    if (!map_closer.loadReferencePoses(poses_path.string())) {
      if (require_exists) {
        std::cerr << "Warning: Could not load reference poses from " << poses_path << '\n';
        std::cerr << "Localization will work but without global pose correction" << '\n';
      } else {
        std::cout << "Warning: Could not load reference poses, continuing with "
                     "empty pose graph"
                  << '\n';
      }
    }

    // Try to load local map points
    if (!loadLocalMapPoints(points_path.string(), local_map_points)) {
      if (require_exists) {
        std::cerr << "Warning: Could not load local map points. "
                  << "ICP refinement will be disabled" << '\n';
      } else {
        std::cout << "Warning: Could not load local map points" << '\n';
      }
    }

    // PLY stores points in map frame; transform back to local keypose frames
    // for internal use (ICP refinement, query building).
    const auto& ref_poses = map_closer.getReferencePoses();
    if (!ref_poses.empty()) {
      for (auto& [map_id, points] : local_map_points) {
        auto pose_it = ref_poses.find(map_id);
        if (pose_it == ref_poses.end()) {
          continue;
        }
        const Eigen::Matrix3d r_inv = pose_it->second.block<3, 3>(0, 0).transpose();
        const Eigen::Vector3d t = pose_it->second.block<3, 1>(0, 3);
        for (auto& pt : points) {
          pt = r_inv * (pt - t);
        }
      }
    }
  }

  return result;
}

// Load PLY-format local map points (binary little-endian, x/y/z double + map_id int32)
static bool loadLocalMapPointsPly(std::ifstream& file,
                                  std::unordered_map<int, std::vector<Eigen::Vector3d>>& out) {
  // Parse header (we already consumed the "ply\n" line via magic check — re-read from start)
  file.seekg(0);

  std::string line;
  uint64_t num_vertices = 0;

  while (std::getline(file, line)) {
    if (line.rfind("element vertex ", 0) == 0) {
      num_vertices = std::stoull(line.substr(15));
    }
    if (line == "end_header") {
      break;
    }
  }

  if (num_vertices == 0) {
    std::cerr << "PLY: no vertices found in header\n";
    return false;
  }

  // Read binary vertex data: 3 doubles + 1 int32 per vertex = 28 bytes
  constexpr size_t VERTEX_SIZE = 3 * sizeof(double) + sizeof(int32_t);

  for (uint64_t i = 0; i < num_vertices; ++i) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int32_t map_id = 0;
    file.read(reinterpret_cast<char *>(&x), sizeof(double));
    file.read(reinterpret_cast<char *>(&y), sizeof(double));
    file.read(reinterpret_cast<char *>(&z), sizeof(double));
    file.read(reinterpret_cast<char *>(&map_id), sizeof(int32_t));

    if (!file) {
      std::cerr << "PLY: unexpected end of file at vertex " << i << '\n';
      return false;
    }
    out[map_id].emplace_back(x, y, z);
  }

  // Suppress unused-variable warning for documentation constant
  (void)VERTEX_SIZE;

  return true;
}

// Load legacy custom binary format
static bool loadLocalMapPointsLegacyBin(
    std::ifstream& file, std::unordered_map<int, std::vector<Eigen::Vector3d>>& out) {
  file.seekg(0);

  uint64_t num_maps = 0;
  file.read(reinterpret_cast<char *>(&num_maps), sizeof(uint64_t));

  for (uint64_t i = 0; i < num_maps; ++i) {
    int32_t map_id = 0;
    file.read(reinterpret_cast<char *>(&map_id), sizeof(int32_t));

    uint64_t num_points = 0;
    file.read(reinterpret_cast<char *>(&num_points), sizeof(uint64_t));

    std::vector<Eigen::Vector3d> points;
    points.reserve(num_points);

    for (uint64_t j = 0; j < num_points; ++j) {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      file.read(reinterpret_cast<char *>(&x), sizeof(double));
      file.read(reinterpret_cast<char *>(&y), sizeof(double));
      file.read(reinterpret_cast<char *>(&z), sizeof(double));
      points.emplace_back(x, y, z);
    }

    if (!file) {
      std::cerr << "Legacy bin: unexpected end of file\n";
      return false;
    }
    out[map_id] = std::move(points);
  }

  return true;
}

bool loadLocalMapPoints(const std::string& points_path,
                        std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points) {
  std::ifstream file(points_path, std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Could not open local map points file: " << points_path << '\n';
    return false;
  }

  try {
    // Auto-detect format via magic bytes
    char magic[4] = {};
    file.read(magic, 4);
    if (!file) {
      std::cerr << "Could not read file header: " << points_path << '\n';
      return false;
    }

    const bool is_ply = (magic[0] == 'p' && magic[1] == 'l' && magic[2] == 'y' && magic[3] == '\n');
    bool ok = false;

    if (is_ply) {
      ok = loadLocalMapPointsPly(file, local_map_points);
    } else {
      ok = loadLocalMapPointsLegacyBin(file, local_map_points);
    }

    file.close();

    if (ok) {
      size_t total_points = 0;
      for (const auto& [id, pts] : local_map_points) {
        total_points += pts.size();
      }
      std::cout << "Loaded " << total_points << " points (" << local_map_points.size()
                << " submaps) from " << points_path << (is_ply ? " (PLY)" : " (legacy bin)")
                << '\n';
    }
    return ok;

  } catch (const std::exception& e) {
    std::cerr << "Error loading local map points: " << e.what() << '\n';
    return false;
  }
}

bool savePosesTum(const std::string& file_path, const LocalMapGraph& local_map_graph) {
  std::ofstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "Could not open file for writing: " << file_path << '\n';
    return false;
  }

  file << std::fixed << std::setprecision(12);

  auto all_ids = local_map_graph.getAllIds();

  for (const uint64_t map_id : all_ids) {
    const Eigen::Matrix4d& pose = local_map_graph[map_id].keypose();
    const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    const Eigen::Vector3d t = pose.block<3, 1>(0, 3);

    // TUM format: timestamp tx ty tz qx qy qz qw (using map_id as timestamp)
    file << map_id << ' ' << t.x() << ' ' << t.y() << ' ' << t.z() << ' ' << q.x() << ' ' << q.y()
         << ' ' << q.z() << ' ' << q.w() << '\n';
  }

  file.close();
  std::cout << "Saved " << all_ids.size() << " poses to " << file_path << " (TUM format)" << '\n';
  return true;
}

bool saveLocalMapPointsPly(
    const std::string& file_path, const LocalMapGraph& local_map_graph,
    const std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points) {
  std::ofstream file(file_path, std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Could not open file for writing: " << file_path << '\n';
    return false;
  }

  // Collect all points with their map IDs
  auto all_ids = local_map_graph.getAllIds();
  std::vector<std::pair<Eigen::Vector3d, int32_t>> all_points;

  for (const uint64_t map_id : all_ids) {
    const auto& local_map = local_map_graph[map_id];
    const std::vector<Eigen::Vector3d> *points_ptr = nullptr;

    if (local_map.hasPointCloud()) {
      points_ptr = &local_map.pointCloud();
    } else {
      auto it = local_map_points.find(static_cast<int>(map_id));
      if (it != local_map_points.end()) {
        points_ptr = &it->second;
      }
    }

    if (points_ptr != nullptr) {
      const auto id = static_cast<int32_t>(map_id);
      const Eigen::Matrix4d& keypose = local_map.keypose();
      const Eigen::Matrix3d r = keypose.block<3, 3>(0, 0);
      const Eigen::Vector3d t = keypose.block<3, 1>(0, 3);

      for (const auto& pt : *points_ptr) {
        all_points.emplace_back(r * pt + t, id);
      }
    }
  }

  // Write PLY header (ASCII)
  std::ostringstream header;
  header << "ply\n";
  header << "format binary_little_endian 1.0\n";
  header << "element vertex " << all_points.size() << "\n";
  header << "property double x\n";
  header << "property double y\n";
  header << "property double z\n";
  header << "property int map_id\n";
  header << "end_header\n";
  const std::string header_str = header.str();
  file.write(header_str.data(), static_cast<std::streamsize>(header_str.size()));

  // Write binary vertex data
  for (const auto& [pt, id] : all_points) {
    double x = pt.x();
    double y = pt.y();
    double z = pt.z();
    file.write(reinterpret_cast<const char *>(&x), sizeof(double));
    file.write(reinterpret_cast<const char *>(&y), sizeof(double));
    file.write(reinterpret_cast<const char *>(&z), sizeof(double));
    file.write(reinterpret_cast<const char *>(&id), sizeof(int32_t));
  }

  file.close();
  std::cout << "Saved " << all_points.size() << " points (" << all_ids.size() << " submaps) to "
            << file_path << " (PLY format)" << '\n';
  return true;
}

bool saveDatabase(const std::string& map_dir, const MapMetadata& metadata,
                  map_closures::MapClosures& map_closer, const LocalMapGraph& local_map_graph,
                  const std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points) {
  try {
    // Create map directory if it doesn't exist
    if (!fs::exists(map_dir)) {
      if (!fs::create_directories(map_dir)) {
        std::cerr << "Failed to create map directory: " << map_dir << '\n';
        return false;
      }
      std::cout << "Created map directory: " << map_dir << '\n';
    }

    // Construct file paths from metadata
    const fs::path db_path = fs::path(map_dir) / metadata.files.database;
    const fs::path poses_path = fs::path(map_dir) / metadata.files.poses;
    const fs::path points_path = fs::path(map_dir) / metadata.files.points;

    // Save the map closures database
    if (!map_closer.save(db_path.string())) {
      std::cerr << "Failed to save map closures database" << '\n';
      return false;
    }
    std::cout << "Saved map closures database to: " << db_path << '\n';

    // Save poses in TUM format
    if (!savePosesTum(poses_path.string(), local_map_graph)) {
      std::cerr << "Failed to save poses TUM file" << '\n';
      return false;
    }

    // Save local map points in PLY format
    if (!saveLocalMapPointsPly(points_path.string(), local_map_graph, local_map_points)) {
      std::cerr << "Failed to save local map points PLY file" << '\n';
      return false;
    }

    // Save metadata
    if (!saveMetadata(map_dir, metadata)) {
      std::cerr << "Failed to save metadata file" << '\n';
      return false;
    }

    std::cout << "Successfully saved complete map database to: " << map_dir << '\n';
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Exception while saving database: " << e.what() << '\n';
    return false;
  }
}

void rebuildLocalMapGraph(
    LocalMapGraph& local_map_graph, const std::unordered_map<int, Eigen::Matrix4d>& reference_poses,
    const std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points) {
  if (reference_poses.empty()) {
    std::cout << "No reference poses to rebuild LocalMapGraph from" << '\n';
    return;
  }

  // Collect and sort map IDs to ensure ordered insertion
  std::vector<int> sorted_ids;
  sorted_ids.reserve(reference_poses.size());
  for (const auto& [id, pose] : reference_poses) {
    sorted_ids.push_back(id);
  }
  std::sort(sorted_ids.begin(), sorted_ids.end());

  // Clear and reinitialize with the first map ID
  const int first_id = sorted_ids.front();
  local_map_graph.clear(first_id);

  // Set the keypose for the first node
  auto first_pose_it = reference_poses.find(first_id);
  if (first_pose_it != reference_poses.end()) {
    local_map_graph.updateKeypose(static_cast<uint64_t>(first_id), first_pose_it->second);
  }

  // Set point cloud for first node if available
  auto first_points_it = local_map_points.find(first_id);
  if (first_points_it != local_map_points.end()) {
    local_map_graph.setPointCloud(static_cast<uint64_t>(first_id), first_points_it->second);
  }

  // Add remaining local maps
  for (size_t i = 1; i < sorted_ids.size(); ++i) {
    const int map_id = sorted_ids[i];
    const auto id = static_cast<uint64_t>(map_id);

    // Get keypose
    auto pose_it = reference_poses.find(map_id);
    if (pose_it == reference_poses.end()) {
      continue;
    }

    // Add the local map with its keypose
    local_map_graph.addLocalMap(id, pose_it->second);

    // Set point cloud if available
    auto points_it = local_map_points.find(map_id);
    if (points_it != local_map_points.end()) {
      local_map_graph.setPointCloud(id, points_it->second);
    }
  }

  std::cout << "Rebuilt LocalMapGraph with " << local_map_graph.size()
            << " local maps from loaded database" << '\n';
}

}  // namespace vegvisir
