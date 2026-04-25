# gpu/ — CUDA + cuVS backends. Compiled only when REMOVERT_BUILD_GPU=ON.
#
# Contents:
#   * src/projection.cu  — fused fill_inf + scan-projection + visibility-update
#                          kernels using libcudacxx cuda::atomic_ref (plan §4.2).
#   * src/knn_gpu.cpp    — cuvs::neighbors::brute_force build/search wrapper,
#                          with a CPU partial_sort fallback when cuvs is absent.
#
# Both code paths are written but not yet GPU-tested (see TODO(local-test)
# comments in the sources).  CPU-only builds (REMOVERT_BUILD_GPU=OFF) ignore
# this directory entirely.

find_package(CUDAToolkit 12.6 REQUIRED)
# cuVS is RAPIDS, often a conda install. Soft-fail to a meaningful message.
find_package(cuvs 24.10 QUIET)
if(NOT cuvs_FOUND)
    message(WARNING
        "cuvs not found — KNN backend will be a CPU-fallback stub. "
        "Install RAPIDS cuvs (24.10+) for the GPU KNN path.")
endif()

add_library(removert_gpu STATIC
    src/projection.cu
    src/knn_gpu.cpp
)
add_library(removert::gpu ALIAS removert_gpu)

target_include_directories(removert_gpu
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(removert_gpu PUBLIC removert::core CUDA::cudart)
if(cuvs_FOUND)
    target_link_libraries(removert_gpu PUBLIC cuvs::cuvs)
    target_compile_definitions(removert_gpu PUBLIC REMOVERT_HAVE_CUVS=1)
endif()
set_target_properties(removert_gpu PROPERTIES CUDA_STANDARD 17
                                              CUDA_SEPARABLE_COMPILATION ON)
