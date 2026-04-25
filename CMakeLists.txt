# removert v2 — pure-C++ offline removert with strengthened visibility.
# See docs/v2-plan.md for the design charter (no ROS, no middleware).

cmake_minimum_required(VERSION 3.18)
project(removert
    VERSION 2.0.0
    LANGUAGES CXX
)

# ---------------------------------------------------------------------------
# Build options
# ---------------------------------------------------------------------------

option(REMOVERT_BUILD_GPU      "Build CUDA/cuVS GPU backends (P5+)" OFF)
option(REMOVERT_BUILD_TESTS    "Build GoogleTest unit tests"        ON)
option(REMOVERT_BUILD_BENCH    "Build Google Benchmark suite"       OFF)
option(REMOVERT_WITH_SQLITE    "Build SQLite dataset reader"        ON)
option(REMOVERT_WITH_HDF5      "Build HDF5 dataset reader/writer"   ON)
option(REMOVERT_WARN_AS_ERROR  "Treat compiler warnings as errors"  OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Dedicated diagnostic profile (compile but stay strict).
add_compile_options(
    -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor
    -Wold-style-cast -Wcast-align -Wunused -Woverloaded-virtual
    -Wconversion -Wsign-conversion -Wnull-dereference -Wdouble-promotion
    -Wformat=2
)
if(REMOVERT_WARN_AS_ERROR)
    add_compile_options(-Werror)
endif()

# ---------------------------------------------------------------------------
# Required deps
# ---------------------------------------------------------------------------

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ---------------------------------------------------------------------------
# Optional deps (graceful fail with explicit messages)
# ---------------------------------------------------------------------------

if(REMOVERT_WITH_SQLITE)
    find_package(SQLite3)
    if(NOT SQLite3_FOUND)
        message(WARNING "REMOVERT_WITH_SQLITE=ON but SQLite3 not found; disabling.")
        set(REMOVERT_WITH_SQLITE OFF CACHE BOOL "" FORCE)
    endif()
endif()

if(REMOVERT_WITH_HDF5)
    # Use the HDF5 C++ API directly. HighFive can be added as a future
    # optimization (chunked reads, type traits) but is not required.
    find_package(HDF5 COMPONENTS C CXX)
    if(NOT HDF5_FOUND OR NOT HDF5_CXX_FOUND)
        message(WARNING "REMOVERT_WITH_HDF5=ON but HDF5 (C++ bindings) not "
                        "found; disabling. Install libhdf5-dev / libhdf5-cpp.")
        set(REMOVERT_WITH_HDF5 OFF CACHE BOOL "" FORCE)
    endif()
endif()

# nlohmann_json — config_loader uses JSON. Try system install, fall back to
# header bundled with most distros.
find_package(nlohmann_json 3.7 QUIET)
if(NOT nlohmann_json_FOUND)
    find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp)
    if(NLOHMANN_JSON_INCLUDE_DIR)
        add_library(nlohmann_json INTERFACE)
        target_include_directories(nlohmann_json
            INTERFACE ${NLOHMANN_JSON_INCLUDE_DIR})
        add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
        set(nlohmann_json_FOUND TRUE)
    endif()
endif()
if(NOT nlohmann_json_FOUND)
    message(FATAL_ERROR
        "nlohmann_json not found. Install nlohmann-json3-dev or set "
        "NLOHMANN_JSON_INCLUDE_DIR.")
endif()

# ---------------------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------------------

add_subdirectory(core)
add_subdirectory(io)
if(REMOVERT_BUILD_GPU)
    enable_language(CUDA)
    add_subdirectory(gpu)
endif()
add_subdirectory(app/cli)

if(REMOVERT_BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    add_subdirectory(tests)
endif()

if(REMOVERT_BUILD_BENCH)
    find_package(benchmark REQUIRED)
    add_subdirectory(bench)
endif()

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

message(STATUS "")
message(STATUS "removert v2 build configuration")
message(STATUS "  CMAKE_BUILD_TYPE       = ${CMAKE_BUILD_TYPE}")
message(STATUS "  REMOVERT_BUILD_GPU     = ${REMOVERT_BUILD_GPU}")
message(STATUS "  REMOVERT_BUILD_TESTS   = ${REMOVERT_BUILD_TESTS}")
message(STATUS "  REMOVERT_BUILD_BENCH   = ${REMOVERT_BUILD_BENCH}")
message(STATUS "  REMOVERT_WITH_SQLITE   = ${REMOVERT_WITH_SQLITE}")
message(STATUS "  REMOVERT_WITH_HDF5     = ${REMOVERT_WITH_HDF5}")
message(STATUS "")
