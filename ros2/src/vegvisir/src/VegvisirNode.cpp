
// Copyright (c) Sensrad 2026

#include "VegvisirNode.hpp"

#include "VegvisirConfig.hpp"

namespace vegvisir {

VegvisirNode::VegvisirNode() : Node("vegvisir_node") {
  // Declare pointcloud_topic as a required parameter (no default)
  auto pointcloud_topic = this->declare_parameter<std::string>("pointcloud_topic");

  // Configure pointcloud QoS — sensors publish best_effort, rosbags may use
  // reliable
  auto pc_reliability =
      this->declare_parameter<std::string>("pointcloud_qos_reliability", "best_effort");
  rmw_qos_profile_t pc_qos =
      (pc_reliability == "best_effort") ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;

  // Subscribe using the declared parameter value
  pointcloud_sub_.subscribe(this, pointcloud_topic, pc_qos);
  odometry_sub_.subscribe(this, "odometry");

  // Initialize localizer with map database
  std::string map_database_path =
      this->declare_parameter<std::string>("map_database_path", "vegvisir/my_map.db");

  // Declare SLAM mode parameter
  bool slam_mode = this->declare_parameter<bool>("slam_mode", true);
  vegvisir::Mode mode = slam_mode ? vegvisir::Mode::SLAM : vegvisir::Mode::LOCALIZATION;

  // Declare runtime configuration parameters
  vegvisir::VegvisirConfig config;
  config.voxel_size = this->declare_parameter<double>("mapping.voxel_size", config.voxel_size);
  config.splitting_distance_slam = this->declare_parameter<double>(
      "mapping.splitting_distance_slam", config.splitting_distance_slam);
  config.splitting_distance_localization = this->declare_parameter<double>(
      "mapping.splitting_distance_localization", config.splitting_distance_localization);
  config.overlap_threshold =
      this->declare_parameter<double>("closure.overlap_threshold", config.overlap_threshold);
  config.pgo_max_iterations =
      this->declare_parameter<int>("optimization.pgo_max_iterations", config.pgo_max_iterations);
  config.inliers_threshold = static_cast<std::size_t>(this->declare_parameter<int>(
      "closure.inliers_threshold", static_cast<int>(config.inliers_threshold)));

  vegvisir_ = std::make_unique<Vegvisir>(map_database_path, mode, config);

  // Create the synchronizer — ExactTime matches identical timestamps on both
  // topics
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), pointcloud_sub_, odometry_sub_);

  // Register the callback
  sync_->registerCallback(
      std::bind(&VegvisirNode::process, this, std::placeholders::_1, std::placeholders::_2));

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Initialize uncertainty visualization marker publisher
  uncertainty_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("uncertainty_volume", 10);

  // Initialize map point cloud publisher (for SLAM mode visualization)
  auto cloud_qos = rclcpp::QoS(1);
  cloud_qos.transient_local();
  cloud_qos.reliable();
  map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", cloud_qos);

  // Initialize keyposes publisher (for SLAM mode visualization)
  auto keyposes_qos = rclcpp::QoS(1);
  keyposes_qos.transient_local();
  keyposes_qos.reliable();
  keyposes_pub_ = this->create_publisher<nav_msgs::msg::Path>("slam_keyposes", keyposes_qos);

  // Initialize per-segment ground plane publisher
  ground_plane_size_m_ =
      this->declare_parameter<double>("visualization.ground_plane_size_m", ground_plane_size_m_);
  auto ground_planes_qos = rclcpp::QoS(1);
  ground_planes_qos.transient_local();
  ground_planes_qos.reliable();
  ground_planes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "ground_planes", ground_planes_qos);

  // Publish the loaded map point cloud at startup
  publishMapPointCloud(this->now());
  publishKeyposes(this->now());
  publishGroundPlanes(this->now());

  RCLCPP_INFO(get_logger(), "Vegvisir node initialized with synchronized subscribers");
  RCLCPP_INFO(get_logger(), "Using map database: %s", map_database_path.c_str());
  RCLCPP_INFO(get_logger(), "Mode: %s", slam_mode ? "SLAM" : "LOCALIZATION");
  RCLCPP_INFO(get_logger(), "Pointcloud QoS reliability: %s", pc_reliability.c_str());
  RCLCPP_INFO(get_logger(),
              "Config: voxel_size=%.2f, splitting_slam=%.1f, splitting_loc=%.1f, "
              "overlap=%.2f, pgo_iters=%d",
              config.voxel_size, config.splitting_distance_slam,
              config.splitting_distance_localization, config.overlap_threshold,
              config.pgo_max_iterations);
}

std::vector<Eigen::Vector3d> VegvisirNode::pointcloudToEigen(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg) {
  using sensor_msgs::PointCloud2ConstIterator;
  const size_t point_count = cloud_msg->width * cloud_msg->height;

  std::vector<Eigen::Vector3d> points;
  points.reserve(point_count);

  PointCloud2ConstIterator<float> it_x(*cloud_msg, "x");
  PointCloud2ConstIterator<float> it_y(*cloud_msg, "y");
  PointCloud2ConstIterator<float> it_z(*cloud_msg, "z");

  for (size_t i = 0; i < point_count; ++i, ++it_x, ++it_y, ++it_z) {
    points.emplace_back(static_cast<double>(*it_x), static_cast<double>(*it_y),
                        static_cast<double>(*it_z));
  }

  return points;
}

Sophus::SE3d VegvisirNode::odometryToSophus(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odometry_msg) {
  const auto& pose_msg = odometry_msg->pose.pose;
  Eigen::Vector3d translation(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
  Eigen::Quaterniond rotation(pose_msg.orientation.w, pose_msg.orientation.x,
                              pose_msg.orientation.y, pose_msg.orientation.z);
  return Sophus::SE3d(rotation, translation);
}

void VegvisirNode::process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
                           const nav_msgs::msg::Odometry::ConstSharedPtr& odometry_msg) {
  auto points = pointcloudToEigen(pointcloud_msg);
  auto absolute_pose = odometryToSophus(odometry_msg);

  const auto sensor_time_ns =
      static_cast<uint64_t>(rclcpp::Time(pointcloud_msg->header.stamp).nanoseconds());
  vegvisir_->update(points, absolute_pose, sensor_time_ns);

  const auto& stamp = pointcloud_msg->header.stamp;
  broadcastMapToOdom(stamp);
  publishUncertaintyMarker(stamp);
  publishMapPointCloud(stamp);
  publishKeyposes(stamp);
  publishGroundPlanes(stamp);
}

void VegvisirNode::broadcastMapToOdom(const rclcpp::Time& timestamp) {
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp = timestamp;
  ts.header.frame_id = map_frame_;
  ts.child_frame_id = odom_frame_;
  ts.transform = ros_conversions::toTransform(Sophus::SE3d(vegvisir_->getMapToOdomTransform()));
  tf_broadcaster_->sendTransform(ts);
}

void VegvisirNode::publishUncertaintyMarker(const rclcpp::Time& timestamp) {
  Eigen::Matrix<double, 6, 6> P_map_odom = vegvisir_->getCovariance();
  Sophus::SE3d current_odom_base = vegvisir_->getCurrentOdomBase();
  Sophus::SE3d base_in_map(vegvisir_->getBaseInMapFrame());

  // Transform covariance to base_link position via adjoint
  Eigen::Matrix<double, 6, 6> Ad_odom_base = current_odom_base.Adj();
  Eigen::Matrix<double, 6, 6> P_map_base = Ad_odom_base * P_map_odom * Ad_odom_base.transpose();

  // Eigenvalues of position covariance → principal axes of uncertainty
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(P_map_base.block<3, 3>(3, 3));
  Eigen::Vector3d eigenvalues = solver.eigenvalues();

  // 2-sigma (95% confidence) scale per axis
  double scale_x = 2.0 * std::sqrt(std::max(eigenvalues(0), 1e-6));
  double scale_y = 2.0 * std::sqrt(std::max(eigenvalues(1), 1e-6));
  double scale_z = 2.0 * std::sqrt(std::max(eigenvalues(2), 1e-6));

  // Color interpolation: green (<1m) → yellow (1-3m) → red (>3m)
  auto uncertainty_color = [](double avg) -> std_msgs::msg::ColorRGBA {
    std_msgs::msg::ColorRGBA c;
    c.a = 0.3f;
    c.b = 0.0f;
    if (avg < 1.0) {
      c.r = 0.0f;
      c.g = 1.0f;
    } else if (avg < 3.0) {
      c.r = static_cast<float>((avg - 1.0) / 2.0);
      c.g = 1.0f;
    } else {
      c.r = 1.0f;
      c.g = 1.0f - std::min(static_cast<float>((avg - 2.0) / 3.0), 1.0f);
    }
    return c;
  };

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = timestamp;
  marker.header.frame_id = map_frame_;
  marker.ns = "localization_uncertainty";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = ros_conversions::toPose(base_in_map);
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.scale.x = 2.0 * scale_x;
  marker.scale.y = 2.0 * scale_y;
  marker.scale.z = 2.0 * scale_z;
  marker.color = uncertainty_color((scale_x + scale_y + scale_z) / 3.0);
  marker.lifetime = rclcpp::Duration::from_seconds(0.5);

  uncertainty_marker_pub_->publish(marker);
}

void VegvisirNode::publishMapPointCloud(const rclcpp::Time& timestamp) {
  // In localization-only mode, publish the reference map only once
  if (vegvisir_->getMode() != Mode::SLAM && reference_map_published_) {
    return;
  }
  reference_map_published_ = true;

  const auto& local_map_graph = vegvisir_->getLocalMapGraph();
  size_t current_closure_count = vegvisir_->getNumClosures();

  // Count finalized segments (all except lastId)
  size_t finalized_count = 0;
  for (const auto& [id, local_map] : local_map_graph) {
    if (id != local_map_graph.lastId()) {
      ++finalized_count;
    }
  }

  // Nothing changed → early return
  if (finalized_count == published_segment_count_ && current_closure_count == last_closure_count_) {
    return;
  }

  bool closure_occurred = current_closure_count != last_closure_count_;

  if (closure_occurred) {
    // Closure changes keyposes — rebuild entire cache
    cached_map_points_.clear();
    for (const auto& [id, local_map] : local_map_graph) {
      if (id == local_map_graph.lastId()) {
        continue;
      }
      const Sophus::SE3d keypose(local_map.keypose());
      for (const auto& pt : local_map.pointCloud()) {
        cached_map_points_.push_back(keypose * pt);
      }
    }
    RCLCPP_INFO(get_logger(), "Closure detected — rebuilt cache with %zu segments (%zu pts)",
                finalized_count, cached_map_points_.size());
  } else {
    // Only new segments — transform and append
    size_t skipped = 0;
    size_t new_segments = 0;
    for (const auto& [id, local_map] : local_map_graph) {
      if (id == local_map_graph.lastId()) {
        continue;
      }
      if (skipped < published_segment_count_) {
        ++skipped;
        continue;
      }
      const Sophus::SE3d keypose(local_map.keypose());
      for (const auto& pt : local_map.pointCloud()) {
        cached_map_points_.push_back(keypose * pt);
      }
      ++new_segments;
    }
    if (new_segments > 0) {
      RCLCPP_INFO(get_logger(), "Appended %zu new segments to cache (total: %zu, %zu pts)",
                  new_segments, finalized_count, cached_map_points_.size());
    }
  }

  published_segment_count_ = finalized_count;
  last_closure_count_ = current_closure_count;

  // Publish the full cached cloud in map frame
  if (!cached_map_points_.empty()) {
    std_msgs::msg::Header header;
    header.stamp = timestamp;
    header.frame_id = map_frame_;
    map_cloud_pub_->publish(ros_conversions::toPointCloud2(cached_map_points_, header));
  }
}

void VegvisirNode::publishKeyposes(const rclcpp::Time& timestamp) {
  if (vegvisir_->getMode() != Mode::SLAM) {
    return;
  }

  const auto& local_map_graph = vegvisir_->getLocalMapGraph();
  if (local_map_graph.size() <= 1) {
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = timestamp;
  path.header.frame_id = map_frame_;

  for (const auto& [id, local_map] : local_map_graph) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = path.header.frame_id;
    ps.header.stamp = rclcpp::Time(static_cast<int64_t>(local_map.keyposeTimestampNs()));
    ps.pose = ros_conversions::toPose(Sophus::SE3d(local_map.keypose()));
    path.poses.push_back(ps);
  }

  keyposes_pub_->publish(path);
}

void VegvisirNode::publishGroundPlanes(const rclcpp::Time& timestamp) {
  const auto ids = vegvisir_->getAvailableMapIds();
  const size_t current_closure_count = vegvisir_->getNumClosures();

  if (ids.size() == published_ground_plane_count_ &&
      current_closure_count == last_ground_closure_count_) {
    return;
  }

  // Golden-angle hue cycle so adjacent map_ids get visibly distinct colors.
  auto color_for_id = [](int map_id) -> std_msgs::msg::ColorRGBA {
    constexpr float golden_angle_deg = 137.508F;
    constexpr float saturation = 0.8F;
    constexpr float value = 0.9F;
    const float hue = std::fmod(static_cast<float>(map_id) * golden_angle_deg, 360.0F);
    const float chroma = value * saturation;
    const float h_prime = hue / 60.0F;
    const float x = chroma * (1.0F - std::fabs(std::fmod(h_prime, 2.0F) - 1.0F));
    float r = 0;
    float g = 0;
    float b = 0;
    if (h_prime < 1.0F) {
      r = chroma;
      g = x;
    } else if (h_prime < 2.0F) {
      r = x;
      g = chroma;
    } else if (h_prime < 3.0F) {
      g = chroma;
      b = x;
    } else if (h_prime < 4.0F) {
      g = x;
      b = chroma;
    } else if (h_prime < 5.0F) {
      r = x;
      b = chroma;
    } else {
      r = chroma;
      b = x;
    }
    const float m = value - chroma;
    std_msgs::msg::ColorRGBA c;
    c.r = r + m;
    c.g = g + m;
    c.b = b + m;
    c.a = 0.35F;
    return c;
  };

  // local_map_graph_ holds the up-to-date keypose for every segment, including
  // ones that exist in MapClosures' density_maps_ but haven't yet been mirrored
  // into reference_poses_ (which only updates after PGO).
  const auto& local_map_graph = vegvisir_->getLocalMapGraph();

  visualization_msgs::msg::MarkerArray array;
  array.markers.reserve(ids.size());
  for (int map_id : ids) {
    const auto key = static_cast<uint64_t>(map_id);
    if (!local_map_graph.hasLocalMap(key)) {
      continue;
    }
    const Eigen::Matrix4d& keypose = local_map_graph[key].keypose();
    const Eigen::Matrix4d& ground_alignment = vegvisir_->getGroundAlignment(map_id);
    const Eigen::Matrix4d ground_in_world = keypose * ground_alignment.inverse();

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = timestamp;
    marker.header.frame_id = map_frame_;
    marker.ns = "ground_planes";
    marker.id = map_id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = ros_conversions::toPose(Sophus::SE3d(ground_in_world));
    marker.scale.x = ground_plane_size_m_;
    marker.scale.y = ground_plane_size_m_;
    marker.scale.z = 0.05;
    marker.color = color_for_id(map_id);
    marker.lifetime = rclcpp::Duration(0, 0);
    array.markers.push_back(std::move(marker));
  }

  ground_planes_pub_->publish(array);

  published_ground_plane_count_ = ids.size();
  last_ground_closure_count_ = current_closure_count;
}

}  // namespace vegvisir
