Copyright (c) Sensrad 2023-2024

# Perception Interfaces

The `oden_interfaces` package in Sensrad's ROS2 software suite contains custom message definitions used for radar data processing and perception tasks.

## Message Definitions

The package includes the following custom message types:

### Detection.msg
Defines the structure for a single detection created from clustered radar data. It includes parameters like range, azimuth, elevation, radial speed, power, and associated uncertainties, along with detection type and probability.

### Detections.msg
A collection of `Detection` messages. This message type aggregates multiple detections into a list, providing a comprehensive view of all detected objects at a given instance.

### EgoMotion.msg
Captures the ego-motion data of the radar platform, including its validity and 3D velocity components (vel_x, vel_y, vel_z). This message is crucial for understanding the movement dynamics of the radar system.

### GroundPlane.msg
Provides the coefficients of the ground plane in a standard `shape_msgs/Plane` format, accompanied by a validity flag. This message conveys the essential ground plane data in a straightforward manner.

### FreeSpace.msg
Comprises of two arrays: one for azimuth values and the other for the corresponding ranges. It also includes the dimensions of the arrays (number of rows), as well as a validity flag. This message represents the free space of a short history of point clouds.

### ObjectTrack.msg
Defines the message structure for a single object track. Each object has an unique id, a track state,  yaw angle, type, 3D position, 3D velocity and a 3x3 extent matrix.

### MultiObjectTracking.msg
A collection of all object tracks currently maintained by the object tracker.

Each message type is tailored to convey specific radar perception data clearly and effectively, ensuring efficient processing and analysis within the Sensrad ROS2 framework.
