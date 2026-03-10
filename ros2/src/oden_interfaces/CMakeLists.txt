# Copyright (c) 2024-2025 Sensrad

cmake_minimum_required(VERSION 3.5)
project(oden_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(INTERNAL_BUILD "Enable internal-only features" ON)

# Silence CMake warnings for unused variables that may be passed from parent
if(DEFINED BUILD_AS_ROS2_PACKAGE)
  # Variable is defined but not used in this interface package
endif()
if(DEFINED TESTS)
  # Variable is defined but not used in this interface package
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

set(srv_files
    "srv/OdenCtrlResetPerception.srv" "srv/OdenCtrlSetPerceptionParameters.srv"
    "srv/ResetSurveillance.srv" "srv/OdenCtrlSetRoiParameters.srv")

set(msg_files
    "msg/ObjectTrack.msg"
    "msg/MultiObjectTracking.msg"
    "msg/EgoMotion.msg"
    "msg/GroundPlane.msg"
    "msg/Detection.msg"
    "msg/Detections.msg"
    "msg/OccupancyGrid.msg"
    "msg/OccupancyGridVoxel.msg"
    "msg/FreeSpace.msg"
    "msg/OdenControlState.msg"
    "msg/OdenStatistics.msg"
    "msg/RegionOfInterest.msg"
    "msg/Voxel.msg"
    "msg/StaticMap.msg")

# Combine messages and services into a SINGLE call
rosidl_generate_interfaces(
  ${PROJECT_NAME} ${msg_files} ${srv_files} # Include BOTH .msg and .srv files
  DEPENDENCIES std_msgs shape_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
