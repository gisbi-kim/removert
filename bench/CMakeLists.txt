# Google Benchmark stubs.  Real bench cases land alongside the GPU work.
add_executable(removert_bench bench_projection.cpp)
target_link_libraries(removert_bench PRIVATE removert::core benchmark::benchmark)
