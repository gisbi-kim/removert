// Benchmark stub for range-image projection. Real numbers will be measured on
// the user's local machine against the seq01 dataset.

#include "removert/range_image.hpp"
#include "removert/types.hpp"

#include <benchmark/benchmark.h>

#include <random>
#include <vector>

namespace {

std::vector<removert::Point> make_random_points(std::size_t n) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> u(-50.0f, 50.0f);
    std::vector<removert::Point> out(n);
    for (auto& p : out) p = removert::Point{u(rng), u(rng), u(rng), 0.0f};
    return out;
}

}  // namespace

static void BM_project(benchmark::State& state) {
    auto pts = make_random_points(static_cast<std::size_t>(state.range(0)));
    removert::SensorConfig sensor; sensor.vfov_deg = 50.0f; sensor.hfov_deg = 360.0f;
    for (auto _ : state) {
        auto rimg = removert::project_to_range_image(pts, sensor, 1.0f, 4);
        benchmark::DoNotOptimize(rimg);
    }
}
BENCHMARK(BM_project)->Arg(1 << 14)->Arg(1 << 16)->Arg(1 << 18);

BENCHMARK_MAIN();
