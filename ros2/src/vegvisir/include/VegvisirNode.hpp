// Copyright (c) Sensrad 2026

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
#include <visualization_msgs/msg/marker_array.hpp>

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
  void publishGroundPlanes(const rclcpp::Time& timestamp);
  void publishQueryCloud(const rclcpp::Time& timestamp);

  // Active local map's accumulated points in the active keypose's local frame,
  // plus (in localization mode) every ring-buffer submap re-expressed in that
  // same frame. This is the cloud the localization closure pipeline queries
  // against; using it for the live ground plane fit gives a temporally stable
  // estimate that doesn't reset on each cutLocalizationSubmap.
  std::vector<Eigen::Vector3d> buildQueryCloudInActiveAnchor() const;

  // Active keypose lifted into the map frame. SLAM keyposes are already in map;
  // the localization anchor is set in odom and needs tf_map_odom applied.
  Eigen::Matrix4d activeKeyposeInMap() const;

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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ground_planes_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr query_cloud_pub_;

  // Frame IDs
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";

  // Track last published sizes for change detection
  size_t published_segment_count_ = 0;
  size_t last_closure_count_ = 0;

  // Square edge length of the ground-plane slab marker (meters)
  double ground_plane_size_m_ = 20.0;

  // Cached map points pre-transformed into map frame
  std::vector<Eigen::Vector3d> cached_map_points_;

  // Publish reference map point cloud from loaded database (called once at
  // startup)
  bool reference_map_published_ = false;
};

}  // namespace vegvisir
