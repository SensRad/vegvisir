# Copyright (c) Sensrad 2024-2025
#
# oden_interfaces/msg/EgoMotion
#
# A custom message type for Sensrad EgoMotion data. The radar coordinate frame
# is defined as: x: right, y: forward, z: upward.

std_msgs/Header header

bool velocity_is_valid # Ego-motion velocity estimator is valid
bool rotational_rate_is_valid # Ego-motion rotational rate estimator is valid
bool odometry_is_valid # Ego-motion odometry is valid

float32 velocity_x # Velocity in the radar x direction in meter/second
float32 velocity_y # Velocity in the radar y direction in meter/second
float32 velocity_z # Velocity in the radar z direction in meter/second
float32 rotational_rate_x # Angular velocity around the radar x axis in radians/second
float32 rotational_rate_y # Angular velocity around the radar y axis in radians/second
float32 rotational_rate_z # Angular velocity around the radar z axis in radians/second

float32 translation_x # Translation in the radar x direction in meter
float32 translation_y # Translation in the radar y direction in meter
float32 translation_z # Translation in the radar z direction in meter
float32 rotation_quat_x # Rotation in quaternion x component
float32 rotation_quat_y # Rotation in quaternion y component
float32 rotation_quat_z # Rotation in quaternion z component
float32 rotation_quat_w # Rotation in quaternion w component

float32 delta_translation_x # Translation in the radar x direction in meter since last message
float32 delta_translation_y # Translation in the radar y direction in meter since last message
float32 delta_translation_z # Translation in the radar z direction in meter since last message
float32 delta_rotation_quat_x # Rotation in quaternion x component since last message
float32 delta_rotation_quat_y # Rotation in quaternion y component since last message
float32 delta_rotation_quat_z # Rotation in quaternion z component since last message
float32 delta_rotation_quat_w # Rotation in quaternion w component since last message
