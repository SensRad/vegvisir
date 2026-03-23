# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Vegvisir is a SLAM (Simultaneous Localization and Mapping) and localization system for 3D LiDARs and 4D Imaging RADARs. It operates in two modes via a pluggable backend architecture: **SLAM** (builds maps incrementally with pose graph optimization) and **Localization** (queries against prebuilt maps with Kalman filtering). The C++ core has Python bindings via pybind11 and a ROS2 node interface.

## Build Commands

### ROS2 nodes (built via colcon)
```bash
cd ros2
vcs import src < vegvisir.repos   # fetches kiss-icp into src/third_party/
colcon build --packages-select vegvisir vegvisir_bringup
source install/setup.bash
```

### Python package (standalone, no ROS2 required)
```bash
cd python
pip install .
```
The Python build uses scikit-build-core and pulls in the C++ libraries via `add_subdirectory` from `cpp/`.

### C++ core only (standalone)
```bash
mkdir -p cpp/build && cd cpp/build
cmake ..
make
```

### Run the ROS2 node
```bash
# Minimal: vegvisir node only (requires external odometry source)
ros2 launch vegvisir vegvisir.launch.py map_path:=sensrad_maps/my_map slam_mode:=true pointcloud_topic:=extended_point_cloud

# Full bringup: auto-launches KISS-ICP if no odometry_topic provided
ros2 launch vegvisir_bringup vegvisir_bringup.launch.py pointcloud_topic:=extended_point_cloud slam_mode:=true
```

### CI
GitHub Actions workflow (`.github/workflows/ros2.yml`) builds the ROS2 packages against both Humble and Jazzy.

## Architecture

### Backend Pattern
`Vegvisir` (core engine) delegates mode-specific logic to `VegvisirBackend` (abstract interface) with two implementations:
- **SlamBackend** — incremental map building, pose graph optimization, loop closure integration
- **LocalizationBackend** — queries prebuilt maps, Kalman filter pose refinement, ring buffer of submaps (MAX_LOCALIZATION_SUBMAPS=8)

The pipeline per update cycle: `preIntegrate()` → voxel integration → `postIntegrate()` → distance check → `runQueryCycle()` → closure detection → ICP verification → `applyAcceptedClosure()`.

### Library Dependency Graph
```
vegvisir (core)
├── voxel_map        (spatial index, tsl::robin_map)
├── map_closures     (loop closure: SIFT + LBD features, density maps, RANSAC 2D)
├── pgo              (g2o pose graph optimizer with CHOLMOD, GNSS constraints)
├── icp_svd          (point-to-point ICP)
├── Sophus           (SE3 Lie group)
└── TBB              (parallelism)
```

### Key Source Locations
- C++ core CMake: `cpp/CMakeLists.txt`
- Core engine: `cpp/include/Vegvisir.hpp`, `cpp/src/Vegvisir.cpp`
- Backend interface: `cpp/include/VegvisirBackend.hpp`
- SLAM backend: `cpp/include/SlamBackend.hpp`
- Localization backend: `cpp/include/LocalizationBackend.hpp`
- Pose graph: `cpp/include/LocalMapGraph.hpp` — `LocalMap` = keyframe + trajectory segment + point cloud
- Map I/O: `cpp/include/VegvisirIO.hpp` — serialization of maps, metadata, GNSS data
- Loop closures: `cpp/map_closures/MapClosures.hpp`
- PGO: `cpp/pgo/pose_graph_optimizer.hpp` (C++20, g2o backend)
- ROS2 node: `ros2/src/vegvisir/include/VegvisirNode.hpp` — subscribes to PointCloud2 + Odometry with ApproximateTime sync
- ROS2 bringup: `ros2/src/vegvisir_bringup/launch/` — launch files for vegvisir + optional KISS-ICP
- Python wrapper: `python/vegvisir/vegvisir.py`
- Pybind11 bindings: `python/vegvisir/pybind/vegvisir_pybind.cpp`

### Point Cloud Format
Input is Nx7: `[x, y, z, range, dist_to_ground, motion_status, occluded]`. Filtering is controlled by constants in `cpp/include/Vegvisir.hpp` (MIN_RANGE, MAX_RANGE, GROUND_PLANE_THRESHOLD, MOTIONSTATUS, OCCLUSION_STATUS).

### Map Database
Saved to the `map_database_path` directory: `metadata.yaml`, `map_closures.db`, `keyposes.tum`, `points.ply`.

## C++ Standards
- Core libraries: C++17 default
- PGO library and map_closures: C++20
- Compiler warnings: `-Wall -Wextra -Wpedantic`

## Fetched Dependencies (via CMake FetchContent)
- **g2o** + SuiteSparse/CHOLMOD — `cpp/cmake/FetchG2O.cmake`
- **Sophus** — `cpp/cmake/FetchSophus.cmake`
- **tsl::robin_map** — `cpp/cmake/FetchTessil.cmake`

## System Dependencies (must be installed)
- **Eigen3**, **OpenCV** (SIFT/LBD features), **TBB** (parallelism)
