# Copyright (c) Sensrad 2025-2026
#
# This module fetches the Sophus library (needed by kiss_icp_core). After
# including this file, the following target is available: - Sophus::Sophus

include(FetchContent)

set(SOPHUS_USE_BASIC_LOGGING
    ON
    CACHE BOOL "Don't use fmt for Sophus library")
set(BUILD_SOPHUS_TESTS
    OFF
    CACHE BOOL "Don't build Sophus tests")
set(BUILD_SOPHUS_EXAMPLES
    OFF
    CACHE BOOL "Don't build Sophus Examples")

# lint_cmake: -readability/wonkycase
FetchContent_Declare(
  sophus
  GIT_REPOSITORY https://github.com/strasdat/Sophus.git
  GIT_TAG 1.24.6)
# lint_cmake: -readability/wonkycase
FetchContent_MakeAvailable(sophus)
