// Copyright (c) Sensrad 2025-2026

#include "VegvisirIO.hpp"

#include <cmath>

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
  fs::path metadata_path = fs::path(map_dir) / "metadata.yaml";
  std::ofstream file(metadata_path);

  if (!file.is_open()) {
    std::cerr << "Could not open metadata file for writing: " << metadata_path << std::endl;
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
    const auto& T = metadata.gnss_anchor_transform;
    for (int i = 0; i < 4; ++i) {
      file << "  - [" << T(i, 0) << ", " << T(i, 1) << ", " << T(i, 2) << ", " << T(i, 3) << "]\n";
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
  std::cout << "Saved metadata to: " << metadata_path << std::endl;
  return true;
}

bool loadMetadata(const std::string& map_dir, MapMetadata& metadata) {
  fs::path metadata_path = fs::path(map_dir) / "metadata.yaml";
  std::ifstream file(metadata_path);

  if (!file.is_open()) {
    std::cerr << "Could not open metadata file: " << metadata_path << std::endl;
    return false;
  }

  // Simple YAML parser for our specific format
  std::string line;
  std::regex key_value_regex(R"(^\s*(\w+):\s*\"?([^\"]*)\"?\s*$)");
  std::regex nested_key_value_regex(R"(^\s{2}(\w+):\s*\"?([^\"]*)\"?\s*$)");
  std::regex array_row_regex(R"(^\s+-\s*\[([^\]]+)\]\s*$)");

  enum class Section { NONE, FILES, GNSS_TRANSFORM, GNSS_ORIGIN };
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
      std::string values_str = match[1].str();
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
      std::string key = match[1].str();
      std::string value = match[2].str();

      if (key == "database") {
        metadata.files.database = value;
      } else if (key == "points") {
        metadata.files.points = value;
      } else if (key == "poses") {
        metadata.files.poses = value;
      }
    } else if (current_section == Section::GNSS_ORIGIN &&
               std::regex_match(line, match, nested_key_value_regex)) {
      std::string key = match[1].str();
      std::string value = match[2].str();

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
      std::string key = match[1].str();
      std::string value = match[2].str();

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
  std::cout << "Loaded metadata from: " << metadata_path << std::endl;
  std::cout << "  Map name: " << metadata.name << std::endl;
  std::cout << "  Location: " << metadata.location << std::endl;
  if (metadata.has_gnss_anchor) {
    double yaw =
        std::atan2(metadata.gnss_anchor_transform(1, 0), metadata.gnss_anchor_transform(0, 0));
    std::cout << "  GNSS anchor: yaw=" << (yaw * 180.0 / M_PI) << "°" << std::endl;
  }
  return true;
}

DatabaseLoadResult loadDatabase(
    const std::string& map_dir, map_closures::MapClosures& map_closer,
    std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points, bool require_exists) {
  DatabaseLoadResult result;
  result.success = true;
  result.loop_closure_enabled = false;

  // Check if map directory exists
  bool dir_exists = fs::exists(map_dir) && fs::is_directory(map_dir);

  // Load or create default metadata
  MapMetadata metadata;
  if (dir_exists) {
    if (!loadMetadata(map_dir, metadata)) {
      // Use defaults if metadata file doesn't exist
      metadata = createDefaultMetadata(map_dir);
      std::cout << "Using default metadata for map directory" << std::endl;
    }
  } else {
    metadata = createDefaultMetadata(map_dir);
  }

  // Construct file paths from metadata
  fs::path db_path = fs::path(map_dir) / metadata.files.database;
  fs::path poses_path = fs::path(map_dir) / metadata.files.poses;
  fs::path points_path = fs::path(map_dir) / metadata.files.points;

  // Check if database file exists
  bool database_exists = fs::exists(db_path);

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
        std::cerr << "Warning: Could not load reference poses from " << poses_path << std::endl;
        std::cerr << "Localization will work but without global pose correction" << std::endl;
      } else {
        std::cout << "Warning: Could not load reference poses, continuing with "
                     "empty pose graph"
                  << std::endl;
      }
    }

    // Try to load local map points
    if (!loadLocalMapPoints(points_path.string(), local_map_points)) {
      if (require_exists) {
        std::cerr << "Warning: Could not load local map points. "
                  << "ICP refinement will be disabled" << std::endl;
      } else {
        std::cout << "Warning: Could not load local map points" << std::endl;
      }
    }
  }

  return result;
}

bool loadLocalMapPoints(const std::string& points_path,
                        std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points) {
  std::ifstream file(points_path, std::ios::binary);

  if (!file.is_open()) {
    std::cerr << "Could not open local map points file: " << points_path << std::endl;
    return false;
  }

  try {
    // Read number of maps (size_t, 8 bytes)
    uint64_t num_maps;
    file.read(reinterpret_cast<char *>(&num_maps), sizeof(uint64_t));

    std::cout << "Loading " << num_maps << " local map point clouds..." << std::endl;

    // Read each local map's points
    for (uint64_t i = 0; i < num_maps; ++i) {
      // Read map_id (int, 4 bytes)
      int32_t map_id;
      file.read(reinterpret_cast<char *>(&map_id), sizeof(int32_t));

      // Read number of points (size_t, 8 bytes)
      uint64_t num_points;
      file.read(reinterpret_cast<char *>(&num_points), sizeof(uint64_t));

      // Read points (each point is 3 doubles = 24 bytes)
      std::vector<Eigen::Vector3d> points;
      points.reserve(num_points);

      for (uint64_t j = 0; j < num_points; ++j) {
        double x, y, z;
        file.read(reinterpret_cast<char *>(&x), sizeof(double));
        file.read(reinterpret_cast<char *>(&y), sizeof(double));
        file.read(reinterpret_cast<char *>(&z), sizeof(double));
        points.emplace_back(x, y, z);
      }

      local_map_points[map_id] = std::move(points);
    }

    file.close();
    std::cout << "Successfully loaded " << local_map_points.size() << " local map point clouds"
              << std::endl;
    return true;

  } catch (const std::exception& e) {
    std::cerr << "Error loading local map points: " << e.what() << std::endl;
    return false;
  }
}

bool savePosesBinary(const std::string& file_path, const LocalMapGraph& local_map_graph) {
  // Save poses in C++ readable binary format
  std::ofstream file(file_path, std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Could not open file for writing: " << file_path << std::endl;
    return false;
  }

  // Collect poses from local_map_graph
  auto all_ids = local_map_graph.getAllIds();
  uint64_t num_poses = all_ids.size();

  // Write number of poses (size_t, 8 bytes)
  file.write(reinterpret_cast<const char *>(&num_poses), sizeof(uint64_t));

  // Write each pose
  for (uint64_t map_id : all_ids) {
    const auto& local_map = local_map_graph[map_id];

    // Write map_id (int, 4 bytes)
    int32_t id = static_cast<int32_t>(map_id);
    file.write(reinterpret_cast<const char *>(&id), sizeof(int32_t));

    // Write 4x4 matrix in row-major order (16 doubles, 128 bytes)
    const Eigen::Matrix4d& pose = local_map.keypose();
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        double val = pose(i, j);
        file.write(reinterpret_cast<const char *>(&val), sizeof(double));
      }
    }
  }

  file.close();
  std::cout << "Saved " << num_poses << " poses to " << file_path << " (binary format)"
            << std::endl;
  return true;
}

bool saveLocalMapPointsBinary(
    const std::string& file_path, const LocalMapGraph& local_map_graph,
    const std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points) {
  // Save local map point clouds in C++ readable binary format
  std::ofstream file(file_path, std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Could not open file for writing: " << file_path << std::endl;
    return false;
  }

  // Get all local map IDs
  auto all_ids = local_map_graph.getAllIds();
  uint64_t num_maps = all_ids.size();

  // Write number of local maps (size_t, 8 bytes)
  file.write(reinterpret_cast<const char *>(&num_maps), sizeof(uint64_t));

  // Write each local map's points
  for (uint64_t map_id : all_ids) {
    // Write map_id (int, 4 bytes)
    int32_t id = static_cast<int32_t>(map_id);
    file.write(reinterpret_cast<const char *>(&id), sizeof(int32_t));

    // Get point cloud - prefer from LocalMapGraph, fallback to
    // local_map_points
    std::vector<Eigen::Vector3d> points;
    const auto& local_map = local_map_graph[map_id];
    if (local_map.hasPointCloud()) {
      points = local_map.pointCloud();
    } else {
      auto it = local_map_points.find(static_cast<int>(map_id));
      if (it != local_map_points.end()) {
        points = it->second;
      }
    }
    // else: empty point cloud

    // Write number of points (size_t, 8 bytes)
    uint64_t num_points = points.size();
    file.write(reinterpret_cast<const char *>(&num_points), sizeof(uint64_t));

    // Write points (each point is 3 doubles = 24 bytes)
    for (const auto& point : points) {
      double x = point.x();
      double y = point.y();
      double z = point.z();
      file.write(reinterpret_cast<const char *>(&x), sizeof(double));
      file.write(reinterpret_cast<const char *>(&y), sizeof(double));
      file.write(reinterpret_cast<const char *>(&z), sizeof(double));
    }
  }

  file.close();
  std::cout << "Saved " << num_maps << " local map point clouds to " << file_path
            << " (binary format)" << std::endl;
  return true;
}

bool saveDatabase(const std::string& map_dir, const MapMetadata& metadata,
                  map_closures::MapClosures& map_closer, const LocalMapGraph& local_map_graph,
                  const std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points) {
  try {
    // Create map directory if it doesn't exist
    if (!fs::exists(map_dir)) {
      if (!fs::create_directories(map_dir)) {
        std::cerr << "Failed to create map directory: " << map_dir << std::endl;
        return false;
      }
      std::cout << "Created map directory: " << map_dir << std::endl;
    }

    // Construct file paths from metadata
    fs::path db_path = fs::path(map_dir) / metadata.files.database;
    fs::path poses_path = fs::path(map_dir) / metadata.files.poses;
    fs::path points_path = fs::path(map_dir) / metadata.files.points;

    // Save the map closures database
    if (!map_closer.save(db_path.string())) {
      std::cerr << "Failed to save map closures database" << std::endl;
      return false;
    }
    std::cout << "Saved map closures database to: " << db_path << std::endl;

    // Save poses in binary format
    if (!savePosesBinary(poses_path.string(), local_map_graph)) {
      std::cerr << "Failed to save poses binary file" << std::endl;
      return false;
    }

    // Save local map points in binary format
    if (!saveLocalMapPointsBinary(points_path.string(), local_map_graph, local_map_points)) {
      std::cerr << "Failed to save local map points binary file" << std::endl;
      return false;
    }

    // Save metadata
    if (!saveMetadata(map_dir, metadata)) {
      std::cerr << "Failed to save metadata file" << std::endl;
      return false;
    }

    std::cout << "Successfully saved complete map database to: " << map_dir << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Exception while saving database: " << e.what() << std::endl;
    return false;
  }
}

void rebuildLocalMapGraph(
    LocalMapGraph& local_map_graph, const std::unordered_map<int, Eigen::Matrix4d>& reference_poses,
    const std::unordered_map<int, std::vector<Eigen::Vector3d>>& local_map_points) {
  if (reference_poses.empty()) {
    std::cout << "No reference poses to rebuild LocalMapGraph from" << std::endl;
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
  int first_id = sorted_ids.front();
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
    int map_id = sorted_ids[i];
    uint64_t id = static_cast<uint64_t>(map_id);

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
            << " local maps from loaded database" << std::endl;
}

}  // namespace vegvisir
