# Copyright (c) Sensrad 2023-2025
#
# oden_interfaces/msg/Detections
#
# A custom message type for Sensrad list of object detections.

std_msgs/Header header

Detection[] detection_list # List of detections (clusters)
