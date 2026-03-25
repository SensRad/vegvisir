# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Vegvisir is a SLAM (Simultaneous Localization and Mapping) and localization system for 3D LiDARs and 4D Imaging RADARs. It operates in two modes via a pluggable backend architecture: **SLAM** (builds maps incrementally with pose graph optimization) and **Localization** (queries against prebuilt maps with Kalman filtering). The C++ core has Python bindings via pybind11 and a ROS2 node interface.

## Build Commands

### C++ core only (standalone)
```bash
mkdir -p cpp/build && cd cpp/build
cmake ..
make
```

### Python package (standalone, no ROS2 required)
```bash
cd python
pip install .
```
The package is named `sensrad-vegvisir`. The build uses scikit-build-core and pulls in C++ libraries via `add_subdirectory` from `cpp/`. Requires Python >=3.12.

### ROS2 nodes (built via colcon)
```bash
cd ros2
vcs import src < vegvisir.repos   # fetches kiss-icp into src/third_party/
colcon build --packages-select vegvisir vegvisir_bringup
source install/setup.bash
```

### Run the ROS2 node
```bash
# Minimal: vegvisir node only (requires external odometry source)
ros2 launch vegvisir vegvisir.launch.py map_path:=sensrad_maps/my_map slam_mode:=true pointcloud_topic:=extended_point_cloud

# Full bringup: auto-launches KISS-ICP if no odometry_topic provided
ros2 launch vegvisir_bringup vegvisir_bringup.launch.py pointcloud_topic:=extended_point_cloud slam_mode:=true
```

## Lint & Format

Python code is checked by CI. Run locally:
```bash
ruff format --check python/
ruff check python/
pylint --rcfile=python/pyproject.toml python/vegvisir/*.py
```
Config is in `python/pyproject.toml`: line-length 100, target Python 3.12, rules E/F/I/W/UP/B.

## CI

GitHub Actions workflows in `.github/workflows/`:
- **ros2.yml** — builds ROS2 packages against Humble and Jazzy containers
- **cpp.yml** — CMake build on ubuntu-24.04/22.04 (x86 + ARM)
- **python.yml** — `pip install ./python/` + import test on the same matrix
- **pylint-black.yml** — ruff format, ruff check, pylint

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
├── icp_svd          (point-to-point ICP via SVD)
├── kalman_filter    (SE3 Kalman filter on Lie group, used by LocalizationBackend)
├── Sophus           (SE3 Lie group)
└── TBB              (parallelism)
```

### Key Source Locations
- Core engine: `cpp/include/Vegvisir.hpp`, `cpp/src/Vegvisir.cpp`
- Backend interface + implementations: `cpp/include/VegvisirBackend.hpp`, `SlamBackend.hpp`, `LocalizationBackend.hpp`
- Tunable config: `cpp/include/VegvisirConfig.hpp` — voxel_size, splitting distances, ICP params, thresholds
- Pose graph structure: `cpp/include/LocalMapGraph.hpp` — `LocalMap` = keyframe + trajectory segment + point cloud
- Map I/O: `cpp/include/VegvisirIO.hpp` — serialization (metadata.yaml, map_closures.db, keyposes.tum, points.ply)
- Pybind11 bindings: `python/vegvisir/pybind/vegvisir_pybind.cpp`
- Python wrappers: `python/vegvisir/vegvisir.py` (main), `map_closures.py`, `voxel_map.py`, `pose_graph_optimizer.py`
- ROS2 node: `ros2/src/vegvisir/include/VegvisirNode.hpp` — PointCloud2 + Odometry with ApproximateTime sync
- ROS2 bringup launch: `ros2/src/vegvisir_bringup/launch/` — vegvisir + optional KISS-ICP
- ROS2 parameters: `ros2/src/vegvisir_bringup/config/vegvisir_params.yaml`

### Point Cloud Format
C++ input is Nx7: `[x, y, z, range, dist_to_ground, motion_status, occluded]`. Python wrapper accepts Nx3 `[x, y, z]`. Filtering is controlled by constants in `cpp/include/Vegvisir.hpp`.

### Analysis Tools
`analyze/` contains standalone Python scripts (no C++ build required):
- `map_analyze.py` — parses map_closures.db binary files, runs all-vs-all feature matching, classifies TP/FP, generates CSV and heatmap plots
- `viz.py` — visualizes points.ply with Open3D, colored by height

## C++ Standards
- **C++20** across all libraries (set in `cpp/CMakeLists.txt`)
- Compiler warnings: `-Wall -Wextra -Wpedantic`

## Fetched Dependencies (via CMake FetchContent)
- **g2o** + SuiteSparse/CHOLMOD — `cpp/cmake/FetchG2O.cmake`
- **Sophus** (1.24.6) — `cpp/cmake/FetchSophus.cmake`
- **tsl::robin_map** (1.4.1) — `cpp/cmake/FetchTessil.cmake`

## System Dependencies (must be installed)
- **Eigen3**, **OpenCV** (SIFT/LBD features), **TBB** (parallelism), **SuiteSparse** (CHOLMOD)
