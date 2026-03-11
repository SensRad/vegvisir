
<div align="center">
    <h1>Vegvisir</h1>
    <a href="https://github.com/SensRad/vegvisir/blob/main/LICENSE"><img src="https://img.shields.io/github/license/SensRad/vegvisir" /></a>
    <br />
    <br />
    <a href="https://sensrad.com">Sensrad</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href=https://github.com/SensRad/vegvisir/issues>Contact Us</a>
  <br />
  <br />

Vegvisir is a SLAM and Localization system that works with both 3D LiDARs and 4D Imaging RADARs.

![mygif2](https://github.com/user-attachments/assets/52290a3c-fe76-4518-8dbb-c12577cd9965)

Vegvisir operates in two modes via a pluggable backend architecture: **SLAM** for incremental map building with pose graph optimization, and **Localization** for querying against prebuilt maps with Kalman filtering. The C++ core has Python bindings via pybind11 and a ROS2 node interface.

</div>

<hr />

## Getting Started

Vegvisir requires an odometry estimate of the vehicles motion to work. Depending on your sensor modality we provide different levels of integration:

- **LiDAR**: We recommend starting with the ROS2 wrapper, which integrates [KISS-ICP](https://github.com/PRBonn/kiss-icp) with Vegvisir. See the [ROS2 README](ros2/README.md) for more information.
- **RADAR**: Vegvisir does not come with any radar odometry algorithms integrated, requiring the user to provide such an estimate. However, Sensrad provides the Oden perception software, which estimates odometry using the **Hugin D1** 4D Imaging Radar.

For commercial inquiries about the [Hugin D1](https://www.sensrad.com/hugin-radar-d1) and the [Oden](https://www.sensrad.com/oden-drive) software, please contact [Sensrad](https://www.sensrad.com) at sales@sensrad.com.

## Acknowledgement

This project is heavily inspired by and initially based upon [KISS-SLAM](https://github.com/PRBonn/kiss-slam). Vegvisir can be seen as a C++ implementation of their Python code. Please cite [their work](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/kiss2025iros.pdf) in your research.

## Contributing

We envision Vegvisir as a community-driven project. If you would like to contribute, please open a Pull Request and we will welcome you to the list of contributors below!

<a href="https://github.com/SensRad/vegvisir/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=SensRad/vegvisir" />
</a>
