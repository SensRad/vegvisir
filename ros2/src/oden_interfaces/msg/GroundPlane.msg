# Copyright (c) Sensrad 2023
#
# oden_interfaces/msg/GroundPlane
#
# A custom message type for Sensrad GroundPlane data

# Header for time and frame information
std_msgs/Header header

# Coefficients of the ground plane according the plane equation a*x+b*y+c*z+d=0
shape_msgs/Plane plane # (a,b,c,d)

# Validity flag
bool is_valid # Boolean reporting if there is a ground plane estimate available
