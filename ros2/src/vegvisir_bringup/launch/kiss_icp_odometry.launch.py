# Copyright (c) Sensrad 2025-2026
"""Reusable launch fragment: KISS-ICP odometry + EgoMotion adapter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("vegvisir_bringup")
    default_kiss_params = os.path.join(bringup_dir, "config", "kiss_icp_params.yaml")

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="sensrad/radar_1/oden",
        description="ROS2 namespace for all nodes",
    )
    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="extended_point_cloud",
        description="Input pointcloud topic for KISS-ICP",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    namespace = LaunchConfiguration("namespace")

    kiss_icp_node = Node(
        package="kiss_icp",
        executable="kiss_icp_node",
        name="kiss_icp_node",
        namespace=namespace,
        output="screen",
        remappings=[
            ("pointcloud_topic", LaunchConfiguration("pointcloud_topic")),
        ],
        parameters=[
            default_kiss_params,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "publish_debug_clouds": True,
            },
        ],
    )

    return LaunchDescription(
        [
            namespace_arg,
            pointcloud_topic_arg,
            use_sim_time_arg,
            kiss_icp_node,
        ]
    )
