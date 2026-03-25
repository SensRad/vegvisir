
<div align="center">
    <h1>Vegvisir</h1>
    <a href="https://github.com/SensRad/vegvisir/blob/master/LICENSE"><img src="https://img.shields.io/github/license/SensRad/vegvisir" /></a>
    <br />
    <br />
    <a href="https://sensrad.com">Sensrad</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://github.com/SensRad/vegvisir/issues>Contact Us</a>
  <br />
  <br />

Vegvisir is a SLAM and Localization system for 3D LiDARs and 4D Imaging RADARs.

![mygif2](https://github.com/user-attachments/assets/52290a3c-fe76-4518-8dbb-c12577cd9965)

</div>

<hr />

## Overview

Vegvisir operates in two modes via a pluggable backend architecture:

- **SLAM** -- incremental map building with pose graph optimization and loop closure detection
- **Localization** -- querying against prebuilt maps with Kalman filter pose refinement

The C++ core provides [Python bindings](python/README.md) via pybind11 and a [ROS2 node](ros2/README.md) interface.

## Getting Started

Vegvisir requires an external odometry estimate. The level of integration depends on your sensor modality:

- **LiDAR** -- The [ROS2 wrapper](ros2/README.md) integrates [KISS-ICP](https://github.com/PRBonn/kiss-icp) as a built-in odometry source.
- **RADAR** -- An external odometry source must be provided. [Sensrad](https://www.sensrad.com) offers the [Oden](https://www.sensrad.com/oden-drive) perception software for the [Hugin D1](https://www.sensrad.com/hugin-radar-d1) 4D Imaging Radar.

For commercial inquiries, contact [Sensrad](https://www.sensrad.com) at sales@sensrad.com.

## Installation

| Interface | Instructions |
|-----------|-------------|
| C++ library | [cpp/README.md](cpp/README.md) |
| Python package | [python/README.md](python/README.md) |
| ROS2 nodes | [ros2/README.md](ros2/README.md) |

## Acknowledgement

This project is inspired by and initially based on [KISS-SLAM](https://github.com/PRBonn/kiss-slam). Please cite [their work](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/kiss2025iros.pdf) in your research.

## Contributing

We welcome contributions. Please open a Pull Request to get started.

<a href="https://github.com/SensRad/vegvisir/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=SensRad/vegvisir" />
</a>
