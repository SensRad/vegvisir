# Copyright (c) Sensrad 2026
"""Reusable launch fragment: vegvisir node only."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("vegvisir_bringup")
    default_params = os.path.join(bringup_dir, "config", "vegvisir_params.yaml")
    default_map_path = os.path.join("vegvisir_maps", "my_map")

    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=default_map_path,
        description="Path to the map directory (contains metadata.yaml and map files)",
    )
    slam_mode_arg = DeclareLaunchArgument(
        "slam_mode",
        default_value="true",
        description="Enable SLAM mode (true) or localization mode (false)",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="ROS2 namespace for vegvisir topics",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="WARN", description="Logging level"
    )
    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        description="Input pointcloud topic (required)",
    )
    odometry_topic_arg = DeclareLaunchArgument(
        "odometry_topic",
        default_value="odometry",
        description="Input odometry topic (nav_msgs/Odometry)",
    )

    vegvisir_node = Node(
        package="vegvisir",
        executable="vegvisir_node",
        name="vegvisir_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        remappings=[
            ("odometry", LaunchConfiguration("odometry_topic")),
        ],
        parameters=[
            default_params,
            {"map_database_path": LaunchConfiguration("map_path")},
            {"slam_mode": LaunchConfiguration("slam_mode")},
            {"pointcloud_topic": LaunchConfiguration("pointcloud_topic")},
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )

    return LaunchDescription(
        [
            map_path_arg,
            slam_mode_arg,
            namespace_arg,
            log_level_arg,
            pointcloud_topic_arg,
            odometry_topic_arg,
            vegvisir_node,
        ]
    )
