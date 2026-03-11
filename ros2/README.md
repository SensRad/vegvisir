# Vegvisir ROS2 Wrapper

The Vegvisir ROS2 wrapper provides launch files and nodes for running Vegvisir with ROS2. It supports both LiDAR (with integrated KISS-ICP odometry) and RADAR (with an external odometry source).

## How to Build

```sh
git clone https://github.com/SensRad/vegvisir
cd ros2
colcon build
source install/setup.bash
```

## How to Run

The ROS2 environment can be configured in a few different ways, depending on your sensor modality and odometry estimate.

To use KISS-ICP as the odometry estimate, launch via:
```sh
ros2 launch vegvisir_bringup vegvisir_kiss_icp.launch.py pointcloud_topic:=<your_PointCloud2_topic>
```

To run Vegvisir with a custom nav_msgs/Odometry message, launch via:
```sh
ros2 launch vegvisir_bringup vegvisir_external_odom.launch.py pointcloud_topic:=<your_PointCloud2_topic> odometry_topic:=<your_Odometry_topic>
```

To run Vegvisir in Localization mode, launch via:
```sh
ros2 launch vegvisir_bringup vegvisir_kiss_icp.launch.py pointcloud_topic:=<your_PointCloud2_topic> slam_mode:=false map_path:=<path_to_vegvisir_map>
```

> **Note:** Localization mode requires a pre-built map. A map can be obtained by running Vegvisir in SLAM mode in the same location.

## Configurable Arguments

| Argument | Description |
|---|---|
| `pointcloud_topic` | Topic with a sensor_msgs/PointCloud2 ROS2 message for Vegvisir to process |
| `odometry_topic` | Topic with an nav_msgs/Odometry ROS2 message for Vegvisir to process |
| `slam_mode` | `true` for SLAM, `false` for Localization (default: `true`) |
| `map_path` | Path to save map in SLAM, and path to load pre-built map in Localization |
