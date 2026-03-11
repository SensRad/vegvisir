# Copyright (c) Sensrad 2025-2026
#
# This module fetches the tessil robin-map header-only library. After including
# this file, the following target is available: - tsl::robin_map

include(FetchContent)

# lint_cmake: -readability/wonkycase
FetchContent_Declare(
  tessil
  GIT_REPOSITORY https://github.com/Tessil/robin-map.git
  GIT_TAG v1.4.1)
# lint_cmake: -readability/wonkycase
FetchContent_MakeAvailable(tessil)
