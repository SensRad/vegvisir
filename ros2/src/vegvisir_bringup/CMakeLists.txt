cmake_minimum_required(VERSION 3.8)
project(vegvisir_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python package (adapter node)
ament_python_install_package(${PROJECT_NAME})

# Install the adapter node executable
install(PROGRAMS scripts/odometry_to_ego_motion
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, config, and rviz directories
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
