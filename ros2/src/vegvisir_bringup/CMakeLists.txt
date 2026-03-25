# Copyright (c) Sensrad 2026

cmake_minimum_required(VERSION 3.8)
project(vegvisir_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch, config, and rviz directories
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_package()
