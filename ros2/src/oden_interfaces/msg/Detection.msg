# Copyright (c) Sensrad 2023
#
# oden_interfaces/msg/Detection
#
# A custom message type for a detection created from clustered data

float32 range # Distance in meter to the center of the cluster.
float32 azimuth # Azimuth angle in radians to the center of the cluster.
float32 elevation # Elevation angle in radians to the center of the cluster.
float32 radial_speed # Radial speed in meter/second of the center of the cluster. A positive value means that the cluster is moving towards the sensor.
float32 power # Mean power in dB of the cluster.
float32 power_std # Standard deviation of power in dB of points included in the cluster.
float32 doppler_std # Standard deviation of doppler in meter/second of points included in the cluster.
float32[3] position # Position in (x,y,z) of the center of the cluster.
float32[9] covariance # Row major order from a 3x3 covariance matrix. Covariance is computed using all 3D points included in the cluster.
uint8 type # A per cluster assessment of object type. Types are defined in "IO_Types.hpp/Enum Type"
float32 type_probability # An assessment on how certain the per cluster type is.
uint32 label_id # An integer defining cluster index.
int32 num_valid_detections # Number of points included in the cluster.
