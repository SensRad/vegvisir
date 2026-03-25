# Vegvisir ROS2 Wrapper

ROS2 nodes and launch files for running Vegvisir with LiDAR (integrated KISS-ICP odometry) or RADAR (external odometry source).

## Building

```bash
git clone https://github.com/SensRad/vegvisir.git
cd vegvisir/ros2
vcs import src < vegvisir.repos
colcon build
source install/setup.bash
```

> **Note:** Install vcstool if needed: `sudo apt install python3-vcstool` or `pip install vcstool`.

## Running

All examples use `vegvisir_bringup`, which handles KISS-ICP, Vegvisir, and RViz in a single launch.

**SLAM with KISS-ICP odometry** (no external odometry needed):
```bash
ros2 launch vegvisir_bringup vegvisir_bringup.launch.py \
  pointcloud_topic:=<your_topic> slam_mode:=true
```

**SLAM with external odometry**:
```bash
ros2 launch vegvisir_bringup vegvisir_bringup.launch.py \
  pointcloud_topic:=<your_topic> odometry_topic:=<your_odom_topic> slam_mode:=true
```

**Localization** against a prebuilt map:
```bash
ros2 launch vegvisir_bringup vegvisir_bringup.launch.py \
  pointcloud_topic:=<your_topic> slam_mode:=false map_path:=<path_to_map>
```

> **Note:** Localization requires a prebuilt map from a prior SLAM run.

**Vegvisir node only** (no KISS-ICP or RViz):
```bash
ros2 launch vegvisir vegvisir.launch.py \
  pointcloud_topic:=<your_topic> slam_mode:=true
```

## Launch Arguments

### vegvisir_bringup

| Argument | Default | Description |
|----------|---------|-------------|
| `pointcloud_topic` | *(required)* | Input `sensor_msgs/PointCloud2` topic |
| `odometry_topic` | `""` | Input `nav_msgs/Odometry` topic; if empty, KISS-ICP is launched |
| `slam_mode` | `true` | `true` for SLAM, `false` for localization |
| `map_path` | `vegvisir_maps/my_map` | Map directory (save in SLAM, load in localization) |
| `namespace` | `""` | ROS2 namespace for all nodes |
| `visualize` | `true` | Launch RViz |
| `rviz_config` | built-in default | Path to custom RViz config |
| `use_sim_time` | `false` | Use simulation clock |
| `log_level` | `WARN` | ROS2 logging level |

### vegvisir (node only)

| Argument | Default | Description |
|----------|---------|-------------|
| `pointcloud_topic` | *(required)* | Input `sensor_msgs/PointCloud2` topic |
| `slam_mode` | `true` | `true` for SLAM, `false` for localization |
| `map_path` | `vegvisir_maps/my_map` | Map directory |
| `log_level` | `WARN` | ROS2 logging level |

## Tuning Parameters

Runtime parameters are configured in `src/vegvisir_bringup/config/vegvisir_params.yaml` and loaded automatically by the bringup launch. Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mapping.voxel_size` | `0.8` | Voxel grid resolution (meters) |
| `mapping.splitting_distance_slam` | `50.0` | Keyframe spacing in SLAM (meters) |
| `mapping.splitting_distance_localization` | `5.0` | Query spacing in localization (meters) |
| `closure.overlap_threshold` | `0.20` | Minimum overlap for valid loop closures |
| `closure.inliers_threshold` | `25` | Minimum inliers for closure acceptance |
| `optimization.pgo_max_iterations` | `10` | Pose graph optimization iterations |
