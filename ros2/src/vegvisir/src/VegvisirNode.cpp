
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

std::vector<Eigen::VectorXd> VegvisirNode::pointcloudToEigen(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg) {
  using sensor_msgs::PointCloud2ConstIterator;
  const size_t point_count = cloud_msg->width * cloud_msg->height;

  std::vector<Eigen::VectorXd> points;
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

  return points;
}

std::pair<Sophus::SE3d, Sophus::SE3d> VegvisirNode::egoMotionToSophus(
    const oden_interfaces::msg::EgoMotion::ConstSharedPtr &odometry_msg) {
  const auto &m = *odometry_msg;

  const auto pose = Sophus::SE3d(
      Eigen::Quaterniond(m.rotation_quat_w, m.rotation_quat_x,
                         m.rotation_quat_y, m.rotation_quat_z),
      Eigen::Vector3d(m.translation_x, m.translation_y, m.translation_z));

  const auto delta_pose = Sophus::SE3d(
      Eigen::Quaterniond(m.delta_rotation_quat_w, m.delta_rotation_quat_x,
                         m.delta_rotation_quat_y, m.delta_rotation_quat_z),
      Eigen::Vector3d(m.delta_translation_x, m.delta_translation_y,
                      m.delta_translation_z));
  return {pose, delta_pose};
}

void VegvisirNode::process(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pointcloud_msg,
    const oden_interfaces::msg::EgoMotion::ConstSharedPtr &odometry_msg) {
  auto points = pointcloudToEigen(pointcloud_msg);
  auto [absolute_pose, delta_pose] = egoMotionToSophus(odometry_msg);

  vegvisir_->update(points, absolute_pose, delta_pose);

  const auto &stamp = pointcloud_msg->header.stamp;
  broadcastMapToOdom(stamp);
  publishUncertaintyMarker(stamp);
  publishMapPointCloud(stamp);
  publishKeyposes(stamp);
}

void VegvisirNode::broadcastMapToOdom(const rclcpp::Time &timestamp) {
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp = timestamp;
  ts.header.frame_id = map_frame_;
  ts.child_frame_id = odom_frame_;
  ts.transform = ros_conversions::toTransform(
      Sophus::SE3d(vegvisir_->getMapToOdomTransform()));
  tf_broadcaster_->sendTransform(ts);
}

void VegvisirNode::publishUncertaintyMarker(const rclcpp::Time &timestamp) {
  Eigen::Matrix<double, 6, 6> P_map_odom = vegvisir_->getCovariance();
  Sophus::SE3d current_odom_base = vegvisir_->getCurrentOdomBase();
  Sophus::SE3d base_in_map(vegvisir_->getBaseInMapFrame());

  // Transform covariance to base_link position via adjoint
  Eigen::Matrix<double, 6, 6> Ad_odom_base = current_odom_base.Adj();
  Eigen::Matrix<double, 6, 6> P_map_base =
      Ad_odom_base * P_map_odom * Ad_odom_base.transpose();

  // Eigenvalues of position covariance → principal axes of uncertainty
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
      P_map_base.block<3, 3>(3, 3));
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

void VegvisirNode::publishMapPointCloud(const rclcpp::Time &timestamp) {
  // In localization-only mode, publish the reference map only once
  if (vegvisir_->getMode() != Mode::SLAM && reference_map_published_) {
    return;
  }
  reference_map_published_ = true;

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

  // Collect all transformed points (skip current/last local map being built)
  std::vector<Eigen::Vector3d> all_points;
  for (const auto &[id, local_map] : local_map_graph) {
    if (id == local_map_graph.lastId()) {
      continue;
    }
    Sophus::SE3d keypose(local_map.keypose());
    for (const auto &pt : local_map.pointCloud()) {
      all_points.push_back(keypose * pt);
    }
  }

  if (all_points.empty()) {
    return;
  }

  std_msgs::msg::Header header;
  header.stamp = timestamp;
  header.frame_id = map_frame_;
  map_cloud_pub_->publish(ros_conversions::toPointCloud2(all_points, header));
  RCLCPP_INFO(get_logger(),
              "Published map cloud with %zu points from %zu local maps",
              all_points.size(), current_local_map_count - 1);
}

void VegvisirNode::publishKeyposes(const rclcpp::Time &timestamp) {
  if (vegvisir_->getMode() != Mode::SLAM) {
    return;
  }

  const auto &local_map_graph = vegvisir_->getLocalMapGraph();
  if (local_map_graph.size() <= 1) {
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = timestamp;
  path.header.frame_id = map_frame_;

  for (const auto &[id, local_map] : local_map_graph) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose = ros_conversions::toPose(Sophus::SE3d(local_map.keypose()));
    path.poses.push_back(ps);
  }

  keyposes_pub_->publish(path);
}

} // namespace vegvisir
