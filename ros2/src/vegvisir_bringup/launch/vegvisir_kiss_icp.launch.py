# Copyright (c) Sensrad 2025-2026
"""Top-level launch: vegvisir + KISS-ICP odometry + optional RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_dir = get_package_share_directory("vegvisir_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    default_rviz = os.path.join(bringup_dir, "rviz", "vegvisir.rviz")

    # Declare all launch arguments
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
    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="extended_point_cloud",
        description="Input pointcloud topic for KISS-ICP",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
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
            "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
        }.items(),
    )

    # Include KISS-ICP + adapter fragment
    kiss_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "kiss_icp_odometry.launch.py")
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
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
            pointcloud_topic_arg,
            use_sim_time_arg,
            visualize_arg,
            rviz_config_arg,
            vegvisir_launch,
            kiss_icp_launch,
            rviz_node,
        ]
    )
