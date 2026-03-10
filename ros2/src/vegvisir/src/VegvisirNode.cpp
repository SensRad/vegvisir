
// Copyright (c) Sensrad 2025-2026

#include "VegvisirNode.hpp"

namespace vegvisir {

VegvisirNode::VegvisirNode()
    : Node("vegvisir_node"), pointcloud_sub_(this, "extended_point_cloud"),
      odometry_sub_(this, "ego_motion") {

  // Initialize localizer with map database
  std::string map_database_path = this->declare_parameter<std::string>(
      "map_database_path", "/home/jacob/data/slam_map4.db");

  // Declare SLAM mode parameter
  bool slam_mode = this->declare_parameter<bool>("slam_mode", false);
  vegvisir::Mode mode =
      slam_mode ? vegvisir::Mode::SLAM : vegvisir::Mode::LOCALIZATION;

  vegvisir_ = std::make_unique<Vegvisir>(map_database_path, mode);

  // Create the synchronizer with a queue size of 10 and max time difference of
  // 100ms
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), pointcloud_sub_, odometry_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));

  // Register the callback
  sync_->registerCallback(std::bind(&VegvisirNode::process, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Initialize uncertainty visualization marker publisher
  uncertainty_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>(
          "uncertainty_volume", 10);

  // Initialize map point cloud publisher (for SLAM mode visualization)
  // Use transient_local QoS so late-joiners (RViz, Foxglove) receive the cloud
  auto cloud_qos = rclcpp::QoS(1);
  cloud_qos.transient_local();
  cloud_qos.reliable();
  map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "map_cloud", cloud_qos);

  // Initialize keyposes publisher (for SLAM mode visualization)
  auto keyposes_qos = rclcpp::QoS(1);
  keyposes_qos.transient_local();
  keyposes_qos.reliable();
  keyposes_pub_ = this->create_publisher<nav_msgs::msg::Path>("slam_keyposes",
                                                              keyposes_qos);

  // Publish the loaded map point cloud at startup
  publishMapPointCloud(this->now());
  publishKeyposes(this->now());

  RCLCPP_INFO(get_logger(),
              "Vegvisir node initialized with synchronized subscribers");
  RCLCPP_INFO(get_logger(), "Using map database: %s",
              map_database_path.c_str());
  RCLCPP_INFO(get_logger(), "Mode: %s", slam_mode ? "SLAM" : "LOCALIZATION");
}

// Function to convert PointCloud2 to Eigen points (only x, y, z)
void VegvisirNode::pointcloudToEigen(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
    std::vector<Eigen::VectorXd> &points) {

  points.clear();

  using sensor_msgs::PointCloud2ConstIterator;
  const size_t point_count = cloud_msg->width * cloud_msg->height;
  points.reserve(point_count);

  PointCloud2ConstIterator<float> it_x(*cloud_msg, "x");
  PointCloud2ConstIterator<float> it_y(*cloud_msg, "y");
  PointCloud2ConstIterator<float> it_z(*cloud_msg, "z");

  for (size_t i = 0; i < point_count; ++i, ++it_x, ++it_y, ++it_z) {
    Eigen::VectorXd point(3);
    point << static_cast<double>(*it_x), static_cast<double>(*it_y),
        static_cast<double>(*it_z);
    points.push_back(std::move(point));
  }
}

// Function to convert EgoMotion to Sophus SE3
void VegvisirNode::egoMotionToSophus(
    const oden_interfaces::msg::EgoMotion::ConstSharedPtr &odometry_msg,
    Sophus::SE3d &absolute_pose, Sophus::SE3d &delta_pose) {
  // Abosolute pose
  Eigen::Vector3d absolute_translation(odometry_msg->translation_x,
                                       odometry_msg->translation_y,
                                       odometry_msg->translation_z);
  Eigen::Quaterniond absolute_rotation(
      odometry_msg->rotation_quat_w, odometry_msg->rotation_quat_x,
      odometry_msg->rotation_quat_y, odometry_msg->rotation_quat_z);
  absolute_pose = Sophus::SE3d(absolute_rotation, absolute_translation);

  // Delta pose
  Eigen::Vector3d delta_translation(odometry_msg->delta_translation_x,
                                    odometry_msg->delta_translation_y,
                                    odometry_msg->delta_translation_z);
  Eigen::Quaterniond delta_rotation(
      odometry_msg->delta_rotation_quat_w, odometry_msg->delta_rotation_quat_x,
      odometry_msg->delta_rotation_quat_y, odometry_msg->delta_rotation_quat_z);
  delta_pose = Sophus::SE3d(delta_rotation, delta_translation);
}

void VegvisirNode::process(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg,
    const oden_interfaces::msg::EgoMotion::ConstSharedPtr &odometry_msg) {
  // Convert messages to Eigen/Sophus formats
  std::vector<Eigen::VectorXd> points;
  pointcloudToEigen(pointcloud_msg, points);

  Sophus::SE3d absolute_pose, delta_pose;
  egoMotionToSophus(odometry_msg, absolute_pose, delta_pose);

  // Call localizer update
  vegvisir_->update(points, absolute_pose, delta_pose);

  // Broadcast map->odom transform from localizer
  broadcastMapToOdom(pointcloud_msg->header.stamp);

  // Publish uncertainty marker from localizer
  publishUncertaintyMarker(pointcloud_msg->header.stamp);

  // Publish map point cloud and keyposes
  publishMapPointCloud(pointcloud_msg->header.stamp);
  publishKeyposes(pointcloud_msg->header.stamp);
}

void VegvisirNode::broadcastMapToOdom(const rclcpp::Time &timestamp) {
  // Get map->odom transform from localizer
  Eigen::Matrix4d tf_map_odom = vegvisir_->getMapToOdomTransform();

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = timestamp;
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  // Extract translation
  transform_stamped.transform.translation.x = tf_map_odom(0, 3);
  transform_stamped.transform.translation.y = tf_map_odom(1, 3);
  transform_stamped.transform.translation.z = tf_map_odom(2, 3);

  // Extract rotation as quaternion
  Eigen::Matrix3d rotation = tf_map_odom.block<3, 3>(0, 0);
  Eigen::Quaterniond quat(rotation);

  transform_stamped.transform.rotation.x = quat.x();
  transform_stamped.transform.rotation.y = quat.y();
  transform_stamped.transform.rotation.z = quat.z();
  transform_stamped.transform.rotation.w = quat.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void VegvisirNode::publishUncertaintyMarker(const rclcpp::Time &timestamp) {
  // Get covariance and pose from localizer
  Eigen::Matrix<double, 6, 6> P_map_odom = vegvisir_->getCovariance();
  Sophus::SE3d current_odom_base = vegvisir_->getCurrentOdomBase();
  Eigen::Matrix4d pose_matrix = vegvisir_->getBaseInMapFrame();

  // Transform covariance to base_link position
  // Using adjoint transformation: P_map_base = Ad(T_odom_base) * P_map_odom *
  // Ad(T_odom_base)^T
  Eigen::Matrix<double, 6, 6> Ad_odom_base = current_odom_base.Adj();
  Eigen::Matrix<double, 6, 6> P_map_base =
      Ad_odom_base * P_map_odom * Ad_odom_base.transpose();

  // Extract position uncertainty (3x3 covariance submatrix)
  // Our Kalman filter uses [rotation; translation] ordering, so translation
  // is at indices [3:5, 3:5]
  Eigen::Matrix3d pos_cov = P_map_base.block<3, 3>(3, 3);

  // Compute eigenvalues to get principal axes of uncertainty ellipsoid
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(pos_cov);
  Eigen::Vector3d eigenvalues = eigen_solver.eigenvalues();

  // Scale to 2-sigma (95% confidence) and ensure positive
  // sigma = sqrt(eigenvalue), 2-sigma = 2*sqrt(eigenvalue)
  double scale_x = 2.0 * std::sqrt(std::max(eigenvalues(0), 1e-6));
  double scale_y = 2.0 * std::sqrt(std::max(eigenvalues(1), 1e-6));
  double scale_z = 2.0 * std::sqrt(std::max(eigenvalues(2), 1e-6));

  // Create shaded uncertainty volume visualization marker
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = timestamp;
  marker.header.frame_id = map_frame_;
  marker.ns = "localization_uncertainty";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Position at base_link
  marker.pose.position.x = pose_matrix(0, 3);
  marker.pose.position.y = pose_matrix(1, 3);
  marker.pose.position.z = pose_matrix(2, 3);
  marker.pose.orientation.w = 1.0;

  // Set scale (diameter of ellipsoid)
  marker.scale.x = 2.0 * scale_x;
  marker.scale.y = 2.0 * scale_y;
  marker.scale.z = 2.0 * scale_z;

  // Compute average uncertainty for color coding
  double avg_uncertainty = (scale_x + scale_y + scale_z) / 3.0;

  // Color based on uncertainty level (green = good, yellow = medium, red =
  // bad) Good: < 1.0m, Medium: 1.0-3.0m, Bad: > 3.0m
  if (avg_uncertainty < 1.0) {
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
  } else if (avg_uncertainty < 3.0) {
    // Interpolate between green and yellow
    float ratio = (avg_uncertainty - 1.0f) / 2.0f;
    marker.color.r = ratio;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
  } else {
    // Interpolate between yellow and red
    float ratio =
        std::min(static_cast<float>((avg_uncertainty - 2.0) / 3.0), 1.0f);
    marker.color.r = 1.0f;
    marker.color.g = 1.0f - ratio;
    marker.color.b = 0.0f;
  }
  marker.color.a = 0.3f; // Semi-transparent

  marker.lifetime = rclcpp::Duration::from_seconds(0.5); // Persist for 0.5s

  uncertainty_marker_pub_->publish(marker);
}

void VegvisirNode::publishMapPointCloud(const rclcpp::Time &timestamp) {

  // In localization-only mode, publish the reference map only once
  if (vegvisir_->getMode() != Mode::SLAM && reference_map_published_) {
    return;
  }
  reference_map_published_ = true;

  // Get local map graph from localizer
  const auto &local_map_graph = vegvisir_->getLocalMapGraph();
  size_t current_local_map_count = local_map_graph.size();
  size_t current_closure_count = vegvisir_->getNumClosures();

  // Only publish if local map count or closure count changed
  if (current_local_map_count == last_local_map_count_ &&
      current_closure_count == last_closure_count_) {
    return;
  }

  last_local_map_count_ = current_local_map_count;
  last_closure_count_ = current_closure_count;

  // Count total points across all local maps (except the current/last one)
  size_t total_points = 0;
  for (const auto &[id, local_map] : local_map_graph) {
    // Skip the last local map (current one being built)
    if (id == local_map_graph.lastId()) {
      continue;
    }
    total_points += local_map.pointCloud().size();
  }

  if (total_points == 0) {
    return;
  }

  // Create PointCloud2 message
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = timestamp;
  cloud_msg.header.frame_id = map_frame_;

  // Set up point cloud fields (x, y, z)
  cloud_msg.height = 1;
  cloud_msg.width = static_cast<uint32_t>(total_points);
  cloud_msg.is_dense = true;
  cloud_msg.is_bigendian = false;

  // Define fields
  sensor_msgs::msg::PointField field_x;
  field_x.name = "x";
  field_x.offset = 0;
  field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_x.count = 1;

  sensor_msgs::msg::PointField field_y;
  field_y.name = "y";
  field_y.offset = 4;
  field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_y.count = 1;

  sensor_msgs::msg::PointField field_z;
  field_z.name = "z";
  field_z.offset = 8;
  field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_z.count = 1;

  cloud_msg.fields = {field_x, field_y, field_z};
  cloud_msg.point_step = 12; // 3 floats * 4 bytes
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step);

  // Fill point cloud data (transform each local map's points by its keypose)
  size_t point_idx = 0;
  for (const auto &[id, local_map] : local_map_graph) {
    // Skip the last local map (current one being built)
    if (id == local_map_graph.lastId()) {
      continue;
    }

    const Eigen::Matrix4d &keypose = local_map.keypose();
    for (const auto &point : local_map.pointCloud()) {
      // Transform point by keypose
      Eigen::Vector4d p_hom(point.x(), point.y(), point.z(), 1.0);
      Eigen::Vector4d p_transformed = keypose * p_hom;

      float x = static_cast<float>(p_transformed.x());
      float y = static_cast<float>(p_transformed.y());
      float z = static_cast<float>(p_transformed.z());

      size_t offset = point_idx * cloud_msg.point_step;
      std::memcpy(&cloud_msg.data[offset], &x, sizeof(float));
      std::memcpy(&cloud_msg.data[offset + 4], &y, sizeof(float));
      std::memcpy(&cloud_msg.data[offset + 8], &z, sizeof(float));

      point_idx++;
    }
  }

  map_cloud_pub_->publish(cloud_msg);
  RCLCPP_INFO(get_logger(),
              "Published map cloud with %zu points from %zu local maps",
              total_points, current_local_map_count - 1);
}

void VegvisirNode::publishKeyposes(const rclcpp::Time &timestamp) {
  // Only publish keyposes in SLAM mode
  if (vegvisir_->getMode() != Mode::SLAM) {
    return;
  }

  // Get local map graph from localizer
  const auto &local_map_graph = vegvisir_->getLocalMapGraph();
  size_t current_local_map_count = local_map_graph.size();

  // Only publish when local map count changes (keyposes only change on closure)
  if (current_local_map_count <= 1) {
    return;
  }

  // Create path message for keyposes
  nav_msgs::msg::Path keyposes_path;
  keyposes_path.header.stamp = timestamp;
  keyposes_path.header.frame_id = map_frame_;

  // Add all keyposes
  for (const auto &[id, local_map] : local_map_graph) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = keyposes_path.header;

    const Eigen::Matrix4d &keypose = local_map.keypose();

    // Extract translation
    pose_stamped.pose.position.x = keypose(0, 3);
    pose_stamped.pose.position.y = keypose(1, 3);
    pose_stamped.pose.position.z = keypose(2, 3);

    // Extract rotation as quaternion
    Eigen::Matrix3d rotation = keypose.block<3, 3>(0, 0);
    Eigen::Quaterniond quat(rotation);

    pose_stamped.pose.orientation.x = quat.x();
    pose_stamped.pose.orientation.y = quat.y();
    pose_stamped.pose.orientation.z = quat.z();
    pose_stamped.pose.orientation.w = quat.w();

    keyposes_path.poses.push_back(pose_stamped);
  }

  keyposes_pub_->publish(keyposes_path);
}

} // namespace vegvisir
