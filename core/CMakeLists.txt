# core/ — pure C++17 algorithm library. No I/O, no ROS, no GPU.

add_library(removert_core STATIC
    src/types_inst.cpp
    src/geometry.cpp
    src/range_image.cpp
    src/remove.cpp
    src/revert.cpp
    src/pipeline.cpp
)
add_library(removert::core ALIAS removert_core)

target_include_directories(removert_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(removert_core PUBLIC Eigen3::Eigen)
target_compile_features(removert_core PUBLIC cxx_std_17)
