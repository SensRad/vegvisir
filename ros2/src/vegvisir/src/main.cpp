// Copyright (c) Sensrad 2025-2026

#include "VegvisirNode.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<vegvisir::VegvisirNode>();
  RCLCPP_INFO(node->get_logger(), "Starting Vegvisir Node...");

  rclcpp::spin(node);

  RCLCPP_INFO(node->get_logger(), "Shutting down Vegvisir node...");

  rclcpp::shutdown();
  return 0;
}
