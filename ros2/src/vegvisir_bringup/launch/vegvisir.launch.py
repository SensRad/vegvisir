# Copyright (c) Sensrad 2025-2026
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
    default_map_path = os.path.join("sensrad_maps", "my_map")

    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=default_map_path,
        description="Path to the map directory (contains metadata.yaml and map files)",
    )
    slam_mode_arg = DeclareLaunchArgument(
        "slam_mode",
        default_value="false",
        description="Enable SLAM mode (true) or localization mode (false)",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="sensrad/radar_1/oden",
        description="ROS2 namespace for vegvisir topics",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="WARN", description="Logging level"
    )
    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="extended_point_cloud",
        description="Input pointcloud topic (remapped to extended_point_cloud)",
    )

    vegvisir_node = Node(
        package="vegvisir",
        executable="vegvisir_node",
        name="vegvisir_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        remappings=[
            ("extended_point_cloud", LaunchConfiguration("pointcloud_topic")),
        ],
        parameters=[
            default_params,
            {"map_database_path": LaunchConfiguration("map_path")},
            {"slam_mode": LaunchConfiguration("slam_mode")},
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
            vegvisir_node,
        ]
    )
