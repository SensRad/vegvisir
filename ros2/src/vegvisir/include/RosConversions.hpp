// Copyright (c) Sensrad 2025-2026

#pragma once

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstring>
#include <vector>

namespace vegvisir::ros_conversions {

inline geometry_msgs::msg::Transform toTransform(const Sophus::SE3d &T) {
  geometry_msgs::msg::Transform tf;
  const auto &t = T.translation();
  const auto &q = T.unit_quaternion();
  tf.translation.x = t.x();
  tf.translation.y = t.y();
  tf.translation.z = t.z();
  tf.rotation.x = q.x();
  tf.rotation.y = q.y();
  tf.rotation.z = q.z();
  tf.rotation.w = q.w();
  return tf;
}

inline geometry_msgs::msg::Pose toPose(const Sophus::SE3d &T) {
  geometry_msgs::msg::Pose pose;
  const auto &t = T.translation();
  const auto &q = T.unit_quaternion();
  pose.position.x = t.x();
  pose.position.y = t.y();
  pose.position.z = t.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

inline sensor_msgs::msg::PointCloud2
toPointCloud2(const std::vector<Eigen::Vector3d> &points,
              const std_msgs::msg::Header &header) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header = header;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size());
  msg.is_dense = true;
  msg.is_bigendian = false;

  // Define x, y, z fields
  constexpr uint32_t float_size = sizeof(float);
  for (uint32_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField field;
    field.name = std::string(1, "xyz"[i]);
    field.offset = i * float_size;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    msg.fields.push_back(field);
  }

  msg.point_step = 3 * float_size;
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);

  for (size_t i = 0; i < points.size(); ++i) {
    const size_t offset = i * msg.point_step;
    const float x = static_cast<float>(points[i].x());
    const float y = static_cast<float>(points[i].y());
    const float z = static_cast<float>(points[i].z());
    std::memcpy(&msg.data[offset], &x, float_size);
    std::memcpy(&msg.data[offset + float_size], &y, float_size);
    std::memcpy(&msg.data[offset + 2 * float_size], &z, float_size);
  }

  return msg;
}

} // namespace vegvisir::ros_conversions
