# Copyright (c) Sensrad 2024-2025
# Best known parameter configuration of Oden perception library. Reflecting most recent sent commands.

#Standard header
std_msgs/Header header

uint8[]  config_identifiers
string[] config_names

uint8 perception_config
string perception_config_name

float32 cluster_scale # Scale factor on the default clustering search radius

float32 elevation_angle # Elevation pitch angle [deg] of the radar relative horizon. Positive angles defined upwards.
float32 vertical_height # Vertical height in meters of the radar installation above the ground

float32 free_space_range
float32 free_space_lower_bound
float32 free_space_upper_bound

uint32 stationary_updates

bool roi_enabled
float32 roi_x_center
float32 roi_y_center
float32 roi_z_center
float32 roi_width
float32 roi_length
float32 roi_height
float32 roi_yaw
float32 roi_pitch
