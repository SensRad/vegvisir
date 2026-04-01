# Copyright (c) Sensrad 2026
#
# This module fetches the tessil robin-map header-only library. After including
# this file, the following target is available: - tsl::robin_map

include(FetchContent)

# lint_cmake: -readability/wonkycase
FetchContent_Declare(
  tessil
  URL https://github.com/Tessil/robin-map/archive/refs/tags/v1.4.1.tar.gz)
# lint_cmake: -readability/wonkycase
FetchContent_MakeAvailable(tessil)
