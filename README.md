# Vegvisir
Vegvisir is a SLAM and Localization system that works with both 3D LiDARs and 4D Imaging RADARs.

![mygif2](https://github.com/user-attachments/assets/52290a3c-fe76-4518-8dbb-c12577cd9965)

Vegvisir operates in two modes via a pluggable backend architecture: **SLAM** for incremental map building with pose graph optimization, and **Localization** for querying against prebuilt maps with Kalman filtering. The C++ core has Python bindings via pybind11 and a ROS2 node interface.

## Getting Started

Vegvisir requires an odometry estimate to work.

- **LiDAR**: We recommend starting with the ROS2 wrapper, which integrates KISS-ICP with Vegvisir. See the ROS2 README for more information.
- **RADAR**: Sensrad provides the Oden perception software, which estimates odometry using the **Hugin D1** 4D Imaging Radar.

For commercial inquiries about the Hugin D1 and the Oden software, please contact Sensrad at sales@sensrad.com.

## Acknowledgement

This project is heavily inspired by and initially based upon KISS-SLAM. Vegvisir can be seen as a C++ implementation of their Python code. Please cite their work in your research.

## Contributing

We envision Vegvisir as a community-driven project. If you would like to contribute, please open a Pull Request and we will welcome you to the list of contributors below!

<a href="https://github.com/SensRad/vegvisir/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=SensRad/vegvisir" />
</a>
