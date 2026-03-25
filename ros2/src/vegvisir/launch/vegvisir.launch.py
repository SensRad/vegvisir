# Copyright (c) Sensrad 2025-2026
"""Launch file for the Vegvisir node"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map_path = os.path.join("vegvisir_maps", "my_map")

    log_level_arg = DeclareLaunchArgument("log_level", default_value="WARN")

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

    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        description="Input pointcloud topic (required)",
    )

    namespace = LaunchConfiguration("namespace", default="")

    vegvisir_node = Node(
        package="vegvisir",
        executable="vegvisir_node",
        name="vegvisir_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {"map_database_path": LaunchConfiguration("map_path")},
            {"slam_mode": LaunchConfiguration("slam_mode")},
            {"pointcloud_topic": LaunchConfiguration("pointcloud_topic")},
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            log_level_arg,
            map_path_arg,
            slam_mode_arg,
            pointcloud_topic_arg,
            vegvisir_node,
        ]
    )
