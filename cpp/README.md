# Vegvisir C++ Core

C++ core libraries for the Vegvisir SLAM and localization system.

## Building

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### System dependencies

- **Eigen3**
- **OpenCV** (SIFT/LBD features in map_closures)

### Fetched automatically (via CMake FetchContent)

- **g2o** + SuiteSparse/CHOLMOD -- pose graph optimization
- **Sophus** -- SE3 Lie group
- **tsl::robin_map** -- fast hash map for voxel indexing

## Library structure

All libraries are compiled as **C++20** (`CMAKE_CXX_STANDARD 20` set at the top level).

```
vegvisir          Core engine, backends, I/O
  voxel_map       Spatial index (voxel grid)
  icp_svd         Point-to-point ICP (SVD)
  map_closures    Loop closure detection (SIFT, LBD)
  pgo             Pose graph optimization (g2o)
  kalman_filter   SE3 Kalman filter
```

### Dependency graph

```
vegvisir
├── voxel_map        -> Eigen3, tsl::robin_map
├── icp_svd          -> voxel_map
├── map_closures     -> Eigen3, OpenCV, Sophus
├── pgo              -> g2o, Eigen3
├── kalman_filter    -> Sophus, Eigen3
├── Sophus
└── Eigen3
```

## Source layout

```
cpp/
├── include/           Public headers (Vegvisir, backends, config, I/O)
├── src/               Core library implementation
├── voxel_map/         Voxel grid with tsl::robin_map
├── icp_svd/           SVD-based point-to-point ICP
├── map_closures/      Loop closure: density maps, SIFT/LBD features, RANSAC 2D
├── pgo/               g2o pose graph optimizer wrapper
├── kalman_filter/     SE3 pose Kalman filter
└── cmake/             FetchContent scripts for g2o, Sophus, tsl::robin_map
```

## Backends

The `Vegvisir` engine delegates mode-specific logic to a `VegvisirBackend`:

- **SlamBackend** -- Incremental map building with pose graph optimization and loop closure.
- **LocalizationBackend** -- Queries prebuilt maps with Kalman filter pose refinement and a ring buffer of submaps.

## Integration

This library is consumed by:

- **Python bindings** (`python/`) via pybind11 and scikit-build-core
- **ROS2 node** (`ros2/src/vegvisir/`) via `add_subdirectory`
