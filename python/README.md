# Vegvisir Python Package

Python bindings for the Vegvisir SLAM and localization engine, built with pybind11.

## Installation

```bash
pip install .
```

Requires Python >= 3.12, a C++20 compiler, and the following system libraries: Eigen3, OpenCV, TBB, SuiteSparse. The C++ core is built automatically via scikit-build-core.

## Usage

```python
import numpy as np
from vegvisir import Vegvisir, VegvisirConfig, Mode

config = VegvisirConfig(voxel_size=0.8, splitting_distance_slam=50.0)
vegvisir = Vegvisir("path/to/map_database", Mode.SLAM, config)

# Feed point clouds (Nx3) and 4x4 SE3 poses
vegvisir.update(points, pose)

# Save the map
vegvisir.save_database()
```

## Modules

| Module | Description |
|--------|-------------|
| `vegvisir.Vegvisir` | Main engine (SLAM and localization) |
| `vegvisir.MapClosures` | Loop closure detection |
| `vegvisir.VoxelMap` | Voxel grid spatial index |
| `vegvisir.PoseGraphOptimizer` | g2o pose graph optimization |
