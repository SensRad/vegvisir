# Vegvisir Python Package

Python bindings for the Vegvisir SLAM/localization engine, built with pybind11.

## Installation

```bash
pip install .
```

Requires a C++20 compiler, Eigen3, and OpenCV. The C++ core libraries are built automatically via scikit-build-core.

## Usage

```python
import numpy as np
from vegvisir import Vegvisir, VegvisirConfig, Mode

config = VegvisirConfig(voxel_size=0.8, splitting_distance_slam=50.0)
v = Vegvisir("path/to/map_database", Mode.SLAM, config)

# Feed point clouds (Nx3) and SE3 poses
v.update(points, pose)

# Save the map
v.save_database()
```

## Modules

- `vegvisir.Vegvisir` -- Main engine (SLAM and localization)
- `vegvisir.MapClosures` -- Loop closure detection
- `vegvisir.VoxelMap` -- Voxel grid spatial index
- `vegvisir.PoseGraphOptimizer` -- g2o pose graph optimization
