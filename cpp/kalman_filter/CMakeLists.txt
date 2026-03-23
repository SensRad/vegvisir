add_library(kalman_filter STATIC PoseKalmanFilter.cpp)
target_include_directories(kalman_filter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(kalman_filter PUBLIC Sophus::Sophus Eigen3::Eigen)
