# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Vegvisir is a SLAM (Simultaneous Localization and Mapping) and localization system. It operates in two modes via a pluggable backend architecture: **SLAM** (builds maps incrementally with pose graph optimization) and **Localization** (queries against prebuilt maps with Kalman filtering). The C++ core has Python bindings via pybind11 and a ROS2 node interface.

## Build Commands

### ROS2 node (built via colcon)
```bash
cd ros2
colcon build --packages-select oden_interfaces vegvisir
source install/setup.bash
```

### Python package (standalone, no ROS2 required)
```bash
cd python
pip install .
```
The Python build uses scikit-build-core and pulls in the C++ libraries via `add_subdirectory` from `cpp/`.

### Run the ROS2 node
```bash
ros2 launch vegvisir vegvisir.launch.py map_path:=sensrad_maps/my_map slam_mode:=true
```

## Architecture

### Backend Pattern
`Vegvisir` (core engine) delegates mode-specific logic to `VegvisirBackend` (abstract interface) with two implementations:
- **SlamBackend** — incremental map building, pose graph optimization, loop closure integration
- **LocalizationBackend** — queries prebuilt maps, Kalman filter pose refinement, ring buffer of submaps

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
- Pose graph: `cpp/include/LocalMapGraph.hpp` — `LocalMap` = keyframe + trajectory segment + point cloud
- Loop closures: `cpp/map_closures/MapClosures.hpp`
- PGO: `cpp/pgo/pose_graph_optimizer.hpp` (C++20, g2o backend)
- ROS2 node: `ros2/src/vegvisir/include/VegvisirNode.hpp` — subscribes to PointCloud2 + EgoMotion with ApproximateTime sync
- Python wrapper: `python/vegvisir/vegvisir.py`
- Pybind11 bindings: `python/vegvisir/pybind/vegvisir_pybind.cpp`
- Custom ROS2 messages: `ros2/src/oden_interfaces/msg/`, `ros2/src/oden_interfaces/srv/`

### Point Cloud Format
Input is Nx7: `[x, y, z, range, dist_to_ground, motion_status, occluded]`. Filtering is controlled by constants in `cpp/include/Vegvisir.hpp` (MIN_RANGE, MAX_RANGE, GROUND_PLANE_THRESHOLD, MOTIONSTATUS, OCCLUSION_STATUS).

### Map Database
Saved to the `map_database_path` directory: `metadata.yaml`, `map_closures.db`, `poses.bin`, `points.bin`, and text reference files.

## C++ Standards
- Core libraries: C++17 default
- PGO library: C++20
- Compiler warnings: `-Wall -Wextra -Wpedantic`

## Fetched Dependencies (via CMake FetchContent)
- **g2o** + SuiteSparse/CHOLMOD — `cpp/cmake/FetchG2O.cmake`
- **Sophus** — `cpp/cmake/FetchSophus.cmake`
- **tsl::robin_map** — `cpp/cmake/FetchTessil.cmake`
