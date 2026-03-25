# Copyright (c) Sensrad 2026
#
# This module fetches g2o and its dependency SuiteSparse if not found on the
# system. After including this file, the following targets are available: -
# g2o::core - g2o::types_slam3d - g2o::solver_cholmod

# lint_cmake: -readability/wonkycase

# Skip install prefix to avoid finding stale/incomplete g2o configs from
# previous builds
find_package(g2o QUIET NO_MODULE NO_CMAKE_INSTALL_PREFIX)

if(NOT TARGET g2o::core)
  message(STATUS "g2o not found on system, using FetchContent")
  include(FetchContent)

  # ===========================================================================
  # SuiteSparse (required by g2o CHOLMOD solver)
  # ===========================================================================
  set(SUITESPARSE_ENABLE_PROJECTS
      "suitesparse_config;amd;camd;ccolamd;colamd;cholmod"
      CACHE STRING "Only build required SuiteSparse components")
  set(SUITESPARSE_USE_PYTHON
      OFF
      CACHE BOOL "")
  set(SUITESPARSE_USE_OPENMP
      OFF
      CACHE BOOL "")
  set(SUITESPARSE_USE_CUDA
      OFF
      CACHE BOOL "")
  set(SUITESPARSE_USE_FORTRAN
      OFF
      CACHE BOOL "")
  set(SUITESPARSE_USE_STRICT
      ON
      CACHE BOOL "")
  set(SUITESPARSE_DEMOS
      OFF
      CACHE BOOL "")
  set(CHOLMOD_SUPERNODAL
      OFF
      CACHE BOOL "")
  set(BUILD_TESTING
      OFF
      CACHE BOOL "")
  set(BUILD_SHARED_LIBS
      OFF
      CACHE BOOL "")
  set(BUILD_STATIC_LIBS
      ON
      CACHE BOOL "")
  set(SuiteSparse_BLAS_integer
      "int64_t"
      CACHE STRING "")

  FetchContent_Declare(
    suitesparse
    SYSTEM EXCLUDE_FROM_ALL
    URL https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/refs/tags/v7.12.2.tar.gz
  )

  # Suppress all warnings from third-party source compilation
  set(_saved_c_flags "${CMAKE_C_FLAGS}")
  set(_saved_cxx_flags "${CMAKE_CXX_FLAGS}")
  set(_saved_msg_level "${CMAKE_MESSAGE_LOG_LEVEL}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
  set(CMAKE_MESSAGE_LOG_LEVEL "ERROR")

  FetchContent_MakeAvailable(suitesparse)

  if(TARGET SuiteSparse::CHOLMOD)
    set(SuiteSparse_CHOLMOD_FOUND
        ON
        CACHE BOOL "")
    set(SuiteSparse_FOUND
        ON
        CACHE BOOL "")
    set(SuiteSparse_NO_CMAKE
        ON
        CACHE BOOL "")
  endif()

  # ===========================================================================
  # g2o
  # ===========================================================================

  # Redirect g2o installs to build dir to prevent polluting ROS2 install space
  # (avoids stale g2oConfig.cmake without g2oTargets.cmake errors)
  set(_saved_install_prefix "${CMAKE_INSTALL_PREFIX}")
  set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/g2o_install_sink")

  # Disable features we don't need (defaults that are already OFF are omitted)
  set(G2O_USE_LOGGING
      OFF
      CACHE BOOL "")
  set(G2O_USE_OPENGL
      OFF
      CACHE BOOL "")
  set(G2O_BUILD_SLAM2D_TYPES
      OFF
      CACHE BOOL "")
  set(G2O_BUILD_SLAM2D_ADDON_TYPES
      OFF
      CACHE BOOL "")
  set(G2O_BUILD_SLAM3D_ADDON_TYPES
      OFF
      CACHE BOOL "")
  set(G2O_BUILD_SCLAM2D_TYPES
      OFF
      CACHE BOOL "")
  set(G2O_BUILD_SBA_TYPES
      OFF
      CACHE BOOL "")
  set(G2O_BUILD_SIM3_TYPES
      OFF
      CACHE BOOL "")
  set(G2O_BUILD_ICP_TYPES
      OFF
      CACHE BOOL "")
  set(G2O_BUILD_DATA_TYPES
      OFF
      CACHE BOOL "")

  FetchContent_Declare(
    g2o
    SYSTEM EXCLUDE_FROM_ALL
    URL https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20241228_git.tar.gz
  )
  FetchContent_MakeAvailable(g2o)

  # Restore install prefix and compiler flags
  set(CMAKE_INSTALL_PREFIX "${_saved_install_prefix}")
  set(CMAKE_C_FLAGS "${_saved_c_flags}")
  set(CMAKE_CXX_FLAGS "${_saved_cxx_flags}")
  set(CMAKE_MESSAGE_LOG_LEVEL "${_saved_msg_level}")

  # Create namespaced aliases (g2o doesn't export these when used as
  # subdirectory)
  add_library(g2o::core ALIAS core)
  add_library(g2o::types_slam3d ALIAS types_slam3d)
  add_library(g2o::solver_cholmod ALIAS solver_cholmod)
else()
  message(STATUS "Using system g2o")
endif()
