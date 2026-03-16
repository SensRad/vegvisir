# Copyright (c) Sensrad 2025-2026
"""Top-level launch: vegvisir + optional KISS-ICP odometry + optional RViz.

If odometry_topic is not provided, KISS-ICP is launched as the default odometry
source. If odometry_topic is provided, it is used directly and KISS-ICP is
skipped. pointcloud_topic is always required.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    bringup_dir = get_package_share_directory("vegvisir_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    pointcloud_topic = LaunchConfiguration("pointcloud_topic").perform(context)
    odometry_topic = LaunchConfiguration("odometry_topic").perform(context)

    actions = []

    if not odometry_topic:
        # No external odometry — launch KISS-ICP and use its output
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "kiss_icp_odometry.launch.py")
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "pointcloud_topic": pointcloud_topic,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            )
        )
        resolved_odometry_topic = "kiss/odometry"
    else:
        resolved_odometry_topic = odometry_topic

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "vegvisir.launch.py")
            ),
            launch_arguments={
                "map_path": LaunchConfiguration("map_path"),
                "slam_mode": LaunchConfiguration("slam_mode"),
                "namespace": LaunchConfiguration("namespace"),
                "log_level": LaunchConfiguration("log_level"),
                "pointcloud_topic": pointcloud_topic,
                "odometry_topic": resolved_odometry_topic,
            }.items(),
        )
    )

    return actions


def generate_launch_description():
    bringup_dir = get_package_share_directory("vegvisir_bringup")
    default_rviz = os.path.join(bringup_dir, "rviz", "vegvisir.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pointcloud_topic",
                description="Input pointcloud topic (required)",
            ),
            DeclareLaunchArgument(
                "odometry_topic",
                default_value="",
                description="Input odometry topic; if empty, KISS-ICP is launched",
            ),
            DeclareLaunchArgument(
                "map_path",
                default_value=os.path.join("sensrad_maps", "my_map"),
                description="Path to the map directory",
            ),
            DeclareLaunchArgument(
                "slam_mode",
                default_value="false",
                description="Enable SLAM mode",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="sensrad/radar_1/oden",
                description="ROS2 namespace",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="WARN",
                description="Logging level",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time",
            ),
            DeclareLaunchArgument(
                "visualize",
                default_value="true",
                description="Launch RViz",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config file",
            ),
            OpaqueFunction(function=_launch_setup),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("visualize")),
            ),
        ]
    )
