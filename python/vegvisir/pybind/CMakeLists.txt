# Copyright (c) Sensrad 2025-2026
pybind11_add_module(vegvisir_pybind MODULE vegvisir_pybind.cpp)
target_link_libraries(vegvisir_pybind PRIVATE vegvisir)
target_include_directories(vegvisir_pybind PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
install(TARGETS vegvisir_pybind LIBRARY DESTINATION .)
