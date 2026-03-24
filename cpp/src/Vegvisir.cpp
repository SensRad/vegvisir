// Copyright (c) Sensrad 2025-2026

#include "Vegvisir.hpp"

#include <iomanip>

#include "LocalizationBackend.hpp"
#include "SlamBackend.hpp"
#include "VegvisirPGO.hpp"

#include <pthread.h>

namespace vegvisir {

Vegvisir::Vegvisir(const std::string& map_database_path, Mode mode, const VegvisirConfig& config)
    : config_(config),
      voxel_grid_(config_.voxel_size),
      tf_map_odom_(Eigen::Matrix4d::Identity()),
      mode_(mode),
      map_database_path_(map_database_path) {
  // Print configuration
  std::cout << "Initializing Vegvisir parameters" << '\n';
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "    Mode: " << (mode_ == Mode::SLAM ? "SLAM" : "LOCALIZATION") << '\n';
  std::cout << "    Map database: " << map_database_path_ << '\n';
  std::cout << "    Voxel size: " << config_.voxel_size << '\n';
  std::cout << "    Splitting distance (SLAM): " << config_.splitting_distance_slam << '\n';
  std::cout << "    Splitting distance (localization): " << config_.splitting_distance_localization
            << '\n';
  std::cout << "    Overlap threshold: " << config_.overlap_threshold << '\n';
  std::cout << "    PGO max iterations: " << config_.pgo_max_iterations << '\n';
  std::cout << "    Inliers threshold: " << config_.inliers_threshold << '\n';

  // Initialize Kalman filter with identity pose (uses default covariances)
  const Sophus::SE3d x0;  // identity
  pose_filter_.init(x0);

  // Initialize transforms
  tf_map_odom_.setIdentity();
  current_odom_base_ = Sophus::SE3d();

  // Initialize metadata with defaults from map directory name
  map_metadata_ = createDefaultMetadata(map_database_path);

  // Initialize MapClosures and load database
  try {
    map_closures::Config config;
    config.density_map_resolution = 0.5F;
    config.density_threshold = 0.05F;
    config.sift_match_ratio = 0.85F;  // Lowe's ratio test for SIFT
    config.density_map_gamma = 0.3F;  // Gamma correction for feature enhancement

    map_closer_ = std::make_unique<map_closures::MapClosures>(config);

    const bool require_exists = (mode_ == Mode::LOCALIZATION);
    auto result = loadDatabase(map_database_path, *map_closer_, local_map_points_, require_exists);

    loop_closure_enabled_ = result.loop_closure_enabled;
    std::cout << result.message << '\n';

    // Try to load metadata (will use defaults if file doesn't exist)
    MapMetadata loaded_metadata;
    if (loadMetadata(map_database_path, loaded_metadata)) {
      map_metadata_ = loaded_metadata;
    }

    // Rebuild the LocalMapGraph from loaded data
    if (result.success && !map_closer_->getReferencePoses().empty()) {
      rebuildLocalMapGraph(local_map_graph_, map_closer_->getReferencePoses(), local_map_points_);
    }

  } catch (const std::exception& e) {
    std::cerr << "Error initializing MapClosures: " << e.what() << '\n';
    loop_closure_enabled_ = false;
  }

  // Select backend. SLAM or Localization
  if (mode_ == Mode::SLAM) {
    backend_ = std::make_unique<SlamBackend>(*this);
  } else {
    backend_ = std::make_unique<LocalizationBackend>(*this);
  }
  backend_->initialize();
}

// Destructor
Vegvisir::~Vegvisir() {
  // Wait for any pending async closure job before saving
  if (closure_future_.valid()) {
    closure_future_.wait();
  }

  // Auto-save database in SLAM mode (skip if already saved explicitly)
  if (mode_ == Mode::SLAM && loop_closure_enabled_ && !map_database_path_.empty() && map_closer_ &&
      !database_saved_) {
    std::cout << "Saving SLAM database on shutdown..." << '\n';
    if (vegvisir::saveDatabase(map_database_path_, map_metadata_, *map_closer_, local_map_graph_,
                               local_map_points_)) {
      std::cout << "Database saved successfully" << '\n';
    } else {
      std::cerr << "Failed to save database" << '\n';
    }
  }
}

void Vegvisir::update(const std::vector<Eigen::Vector3d>& points,
                      const Sophus::SE3d& absolute_pose) {
  if (!loop_closure_enabled_) {
    std::cout << "Loop closure disabled" << '\n';
    return;
  }
  if (!backend_) {
    std::cerr << "No backend initialized" << '\n';
    return;
  }
  // Compute delta from previous absolute poses
  Sophus::SE3d delta_pose;
  if (has_previous_pose_) {
    delta_pose = current_odom_base_.inverse() * absolute_pose;
  }
  has_previous_pose_ = true;

  // Store current base_link pose in odom frame
  const Eigen::Matrix4d pose_odom_base = absolute_pose.matrix();
  current_odom_base_ = absolute_pose;

  // Mode-specific pose estimation: compute current_pose_ + tf_map_odom_
  backend_->updatePoseEstimate(pose_odom_base, delta_pose);

  const std::vector<Eigen::Vector3d> downsampled_points =
      voxel_map::voxelDownsample(points, config_.voxel_size);

  // Integrate into voxel grid, and prune points too far away
  voxel_grid_.integrateFrame(downsampled_points, current_pose_);
  voxel_grid_.pruneFarPoints(current_pose_, LOCAL_MAP_RADIUS_M);

  // Mode-specific trajectory policy
  backend_->updateTrajectory();

  // Update traveled distance
  distance_since_query_ += delta_pose.translation().norm();

  // Query cadence is backend-specific
  if (distance_since_query_ < backend_->queryDistanceM()) {
    return;
  }
  distance_since_query_ = 0.0;

  // Mode-specific query cycle
  backend_->runQueryCycle(pose_odom_base);
}

void Vegvisir::processLoopClosures(int query_id,
                                   const std::vector<Eigen::Vector3d>& query_points_mc,
                                   const std::vector<Eigen::Vector3d>& query_points_icp,
                                   const Eigen::Matrix4d& query_odom_base) {
  if (!backend_ || !map_closer_) {
    return;
  }

  // Retrieve closure candidates from MapClosures
  std::vector<map_closures::ClosureCandidate> closures =
      backend_->retrieveCandidates(query_id, query_points_mc);

  for (auto& closure : closures) {
    // Check minimum inliers and availability of local map points
    if (closure.number_of_inliers < config_.inliers_threshold ||
        !hasLocalMapPoints(closure.source_id)) {
      continue;
    }

    // Read reference points from map
    const auto& reference_points = getLocalMapPoints(closure.source_id);

    // Refine closure pose with ICP
    auto [converged, refined_pose] =
        performICPRefinement(query_points_icp, reference_points, closure.pose);
    // if (!converged) {
    //   continue;
    // }
    //  Validate refined pose using overlap computation
    const bool is_valid = validateClosurePose(query_points_icp, reference_points, refined_pose);
    if (!is_valid) {
      continue;
    }

    // Update the closure with the refined and validated pose
    closure.pose = refined_pose;

    {
      const std::lock_guard<std::mutex> lock(closure_mutex_);

      // Store for visualization/external consumption
      closures_.push_back(closure);

      // Mode-specific application (PGO vs KF)
      backend_->applyAcceptedClosure(closure, query_odom_base);
    }
  }
}

void Vegvisir::processLoopClosuresAsync(int query_id, std::vector<Eigen::Vector3d> query_points_mc,
                                        std::vector<Eigen::Vector3d> query_points_icp,
                                        Eigen::Matrix4d query_odom_base) {
  // Skip if a previous closure job is still running
  if (closure_future_.valid() &&
      closure_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return;
  }

  closure_future_ =
      std::async(std::launch::async, [this, query_id, pts_mc = std::move(query_points_mc),
                                      pts_icp = std::move(query_points_icp), query_odom_base]() {
        // Deprioritize this thread so it doesn't starve the main
        // pipeline
        const struct sched_param param{};
        pthread_setschedparam(pthread_self(), SCHED_BATCH, &param);

        processLoopClosures(query_id, pts_mc, pts_icp, query_odom_base);
      });
}

// ICP refinement and overlap validation
std::pair<bool, Eigen::Matrix4d> Vegvisir::performICPRefinement(
    const std::vector<Eigen::Vector3d>& query_points,
    const std::vector<Eigen::Vector3d>& reference_points,
    const Eigen::Matrix4d& initial_pose) const {
  if (query_points.empty() || reference_points.empty()) {
    return {false, initial_pose};
  }

  // NOLINTNEXTLINE(readability-suspicious-call-argument)
  const icp::Result result = icp::IcpSvd::pointToPointICP(
      reference_points, query_points, initial_pose, config_.icp_refinement_voxel_size,
      config_.icp_max_iterations, config_.icp_convergence_criterion,
      config_.icp_max_correspondence_distance);

  return {result.converged, result.transform};
}

bool Vegvisir::validateClosurePose(const std::vector<Eigen::Vector3d>& query_points,
                                   const std::vector<Eigen::Vector3d>& reference_points,
                                   const Eigen::Matrix4d& pose) const {
  if (query_points.empty() || reference_points.empty()) {
    return false;
  }

  voxel_map::VoxelMap ref_map(config_.voxel_size);
  ref_map.integrateFrame(reference_points, pose);
  const size_t num_source_voxels = ref_map.numVoxels();

  voxel_map::VoxelMap query_map(config_.voxel_size);
  query_map.addPoints(query_points);
  const size_t num_target_voxels = query_map.numVoxels();

  voxel_map::VoxelMap union_map(config_.voxel_size);
  union_map.integrateFrame(reference_points, pose);
  union_map.addPoints(query_points);
  const size_t union_voxels = union_map.numVoxels();

  if (num_source_voxels == 0 || num_target_voxels == 0) {
    return false;
  }

  int intersection = static_cast<int>(num_source_voxels) + static_cast<int>(num_target_voxels) -
                     static_cast<int>(union_voxels);
  if (intersection < 0) {
    intersection = 0;
  }

  const double overlap = static_cast<double>(intersection) /
                         static_cast<double>(std::min(num_source_voxels, num_target_voxels));

  return overlap > config_.overlap_threshold;
}

Eigen::Matrix4d Vegvisir::getBaseInMapFrame() const {
  const Sophus::SE3d t_map_odom(tf_map_odom_);
  const Sophus::SE3d t_map_base = t_map_odom * current_odom_base_;
  return t_map_base.matrix();
}

void Vegvisir::transformAndAppendPoints(const std::vector<Eigen::Vector3d>& in,
                                        const Eigen::Matrix4d& transform_matrix,
                                        std::vector<Eigen::Vector3d>& out) {
  out.reserve(out.size() + in.size());
  for (const auto& p : in) {
    const Eigen::Vector4d hp(p.x(), p.y(), p.z(), 1.0);
    const Eigen::Vector4d tp = transform_matrix * hp;
    out.emplace_back(tp.x(), tp.y(), tp.z());
  }
}

bool Vegvisir::saveDatabase() {
  if (!map_closer_ || mode_ != Mode::SLAM) {
    std::cerr << "Cannot save database: Not in SLAM mode or MapClosures not "
                 "initialized"
              << '\n';
    return false;
  }

  database_saved_ = vegvisir::saveDatabase(map_database_path_, map_metadata_, *map_closer_,
                                           local_map_graph_, local_map_points_);
  return database_saved_;
}

// Access methods for map closure data
const std::unordered_map<int, Eigen::Matrix4d>& Vegvisir::getReferencePoses() const {
  if (!map_closer_) {
    static const std::unordered_map<int, Eigen::Matrix4d> EMPTY;
    return EMPTY;
  }
  return map_closer_->getReferencePoses();
}

const Eigen::Matrix4d& Vegvisir::getReferencePose(int map_id) const {
  return map_closer_->getReferencePose(map_id);
}

const map_closures::DensityMap& Vegvisir::getDensityMap(int map_id) const {
  return map_closer_->getDensityMapFromId(map_id);
}

std::vector<int> Vegvisir::getAvailableMapIds() const {
  if (!map_closer_) {
    return {};
  }
  return map_closer_->getAvailableMapIds();
}

const Eigen::Matrix4d& Vegvisir::getGroundAlignment(int map_id) const {
  return map_closer_->getGroundAlignment(map_id);
}

const std::vector<map_closures::ClosureCandidate>& Vegvisir::getClosures() const {
  return closures_;
}

size_t Vegvisir::getNumClosures() const {
  return closures_.size();
}

// Local map points access methods
bool Vegvisir::hasLocalMapPoints(int map_id) const {
  return local_map_points_.find(map_id) != local_map_points_.end();
}

const std::vector<Eigen::Vector3d>& Vegvisir::getLocalMapPoints(int map_id) const {
  static const std::vector<Eigen::Vector3d> EMPTY;
  const auto it = local_map_points_.find(map_id);
  if (it != local_map_points_.end()) {
    return it->second;
  }
  return EMPTY;
}

std::vector<Eigen::Matrix4d> Vegvisir::fineGrainedOptimizationAndUpdateKeyposes() {
  // First run PGO to get optimized poses
  std::vector<Eigen::Matrix4d> optimized_poses = fineGrainedOptimization();

  if (optimized_poses.empty()) {
    return optimized_poses;
  }

  // Update keyposes to match optimized trajectory
  // PGO vertex layout: for each node, traj.size() poses starting at
  // keypose * traj[0]. Recover keypose as optimized_poses[first] * traj[0]^-1.
  int pose_idx = 0;

  for (auto it = local_map_graph_.cbegin(); it != local_map_graph_.cend(); ++it) {
    const int node_id = static_cast<int>(it->first);
    const auto& node = it->second;
    const auto& traj = node.localTrajectory();

    if (pose_idx >= static_cast<int>(optimized_poses.size())) {
      break;
    }

    const Eigen::Matrix4d new_keypose =
        traj.empty() ? optimized_poses[pose_idx] : optimized_poses[pose_idx] * traj[0].inverse();
    pose_idx += static_cast<int>(traj.size());

    // Update LocalMapGraph keypose
    local_map_graph_.updateKeypose(node_id, new_keypose);

    // Update MapClosures reference pose
    if (map_closer_) {
      map_closer_->setReferencePose(node_id, new_keypose);
    }

    // Point clouds in pcd_ and local_map_points_ are in keypose-local
    // frame. Do NOT re-transform them: the updated keypose automatically
    // gives the corrected world position via keypose * p_local.
  }

  return optimized_poses;
}

std::vector<Eigen::Matrix4d> Vegvisir::fineGrainedOptimization() {
  auto pgo_result = runFineGrainedPGO(local_map_graph_, closures_, gnss_state_, config_);
  return std::move(pgo_result.optimized_poses);
}

}  // namespace vegvisir
