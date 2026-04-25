# gpu/ — CUDA + cuVS backends. Compiled only when REMOVERT_BUILD_GPU=ON.
#
# This is a SKELETON — the real fused projection / cuVS KNN kernels are
# scheduled for P5/P6. We currently provide:
#   * CMake plumbing that reliably finds CUDA + cuVS,
#   * compile-time stubs that satisfy core/'s IRangeImageBackend / IKnnBackend
#     interface (TODO comments mark the actual algorithm sites).

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
