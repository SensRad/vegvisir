# Copyright (c) Sensrad 2023
#
# oden_interfaces/msg/ObjectTrack
#
# A custom message type for Sensrad object tracks

uint32 track_id
uint8 track_state
float32 yaw
uint8 type

float32[3] position
float32[3] velocity
float32[9] extent
