# Copyright (c) Sensrad 2023-2025

# oden_interfaces/msg/FreeSpace

# A custom message type for Sensrad FreeSpace data

# Header for time and frame information
std_msgs/Header header

# Ground polygon vertices (x, y, z) in radar frame
geometry_msgs/Point[] free_space_polygon

# Validity flag
bool is_valid

# Ground plane state (true = ACTIVE, false = NOMINAL)
bool ground_plane_is_valid
