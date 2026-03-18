// Copyright (c) Sensrad 2025-2026

#pragma once

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include "RosConversions.hpp"
#include "Vegvisir.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

namespace vegvisir {

class VegvisirNode : public rclcpp::Node {
 public:
  VegvisirNode();

 private:
  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud_msg,
               const nav_msgs::msg::Odometry::ConstSharedPtr& odometry_msg);

  std::vector<Eigen::Vector3d> pointcloudToEigen(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);

  Sophus::SE3d odometryToSophus(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry_msg);

  void broadcastMapToOdom(const rclcpp::Time& timestamp);
  void publishUncertaintyMarker(const rclcpp::Time& timestamp);
  void publishMapPointCloud(const rclcpp::Time& timestamp);
  void publishKeyposes(const rclcpp::Time& timestamp);

  // Message filter subscribers
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;

  // Vegvisir instance (handles all pose estimation)
  std::unique_ptr<Vegvisir> vegvisir_;

  // Synchronizer
  using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2,
                                                               nav_msgs::msg::Odometry>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr uncertainty_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr keyposes_pub_;

  // Frame IDs
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";

  // Diagnostic timer for detecting sync/QoS issues
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  size_t process_count_ = 0;
  size_t last_diagnostic_count_ = 0;

  // Track last published sizes for change detection
  size_t published_segment_count_ = 0;
  size_t last_closure_count_ = 0;

  // Cached map points pre-transformed into map frame
  std::vector<Eigen::Vector3d> cached_map_points_;

  // Publish reference map point cloud from loaded database (called once at
  // startup)
  bool reference_map_published_ = false;
};

}  // namespace vegvisir
