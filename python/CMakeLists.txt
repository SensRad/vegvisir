# Copyright (c) Sensrad 2025-2026
cmake_minimum_required(VERSION 3.16)
project(vegvisir_pybind LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Dependencies ----
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

# Allow older cmake_minimum_required() in fetched dependencies
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.27)
  cmake_policy(SET CMP0148 OLD)
endif()
if(POLICY CMP0174)
  cmake_policy(SET CMP0174 OLD)
endif()

# ---- Build vegvisir libraries via add_subdirectory ----
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp
                 ${CMAKE_BINARY_DIR}/vegvisir)

# ---- Pybind module ----
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/vegvisir/pybind
                 ${CMAKE_BINARY_DIR}/vegvisir_pybind)
