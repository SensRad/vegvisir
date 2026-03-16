# Copyright (c) Sensrad 2025-2026
"""Top-level launch: vegvisir with user-provided nav_msgs/Odometry + optional RViz.

The user is expected to publish nav_msgs/Odometry on the configured
odometry_topic externally.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("vegvisir_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    default_rviz = os.path.join(bringup_dir, "rviz", "vegvisir.rviz")

    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=os.path.join("sensrad_maps", "my_map"),
        description="Path to the map directory",
    )
    slam_mode_arg = DeclareLaunchArgument(
        "slam_mode", default_value="false", description="Enable SLAM mode"
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="sensrad/radar_1/oden",
        description="ROS2 namespace",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="WARN", description="Logging level"
    )
    odometry_topic_arg = DeclareLaunchArgument(
        "odometry_topic",
        default_value="odometry",
        description="Input odometry topic (nav_msgs/Odometry)",
    )
    visualize_arg = DeclareLaunchArgument(
        "visualize", default_value="true", description="Launch RViz"
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", default_value=default_rviz, description="RViz config file"
    )

    # Include vegvisir fragment
    vegvisir_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "vegvisir.launch.py")
        ),
        launch_arguments={
            "map_path": LaunchConfiguration("map_path"),
            "slam_mode": LaunchConfiguration("slam_mode"),
            "namespace": LaunchConfiguration("namespace"),
            "log_level": LaunchConfiguration("log_level"),
            "odometry_topic": LaunchConfiguration("odometry_topic"),
        }.items(),
    )

    # RViz (conditional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("visualize")),
    )

    return LaunchDescription(
        [
            map_path_arg,
            slam_mode_arg,
            namespace_arg,
            log_level_arg,
            odometry_topic_arg,
            visualize_arg,
            rviz_config_arg,
            vegvisir_launch,
            rviz_node,
        ]
    )
