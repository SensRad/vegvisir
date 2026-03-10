# Copyright (c) Sensrad 2025
#
# oden_interfaces/msg/StaticMap
#
# A custom message type for the static map

std_msgs/Header header
float32 voxel_size
Voxel[] grid_centers
