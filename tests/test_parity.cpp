// CPU vs GPU parity test (plan §4.6).
//
// SKELETON: only enabled when REMOVERT_BUILD_GPU=ON. Until P5 lands the
// fused projection kernel, this only exercises the CPU code path so the
// harness compiles cleanly. The asserts will be tightened (and a real GPU
// invocation added) when the kernel comes online.

#include "removert/range_image.hpp"
#include "removert/types.hpp"

#include <gtest/gtest.h>

#include <random>

#if REMOVERT_BUILD_GPU
#include "removert/gpu/projection.cuh"
#endif

TEST(Parity, RangeImageDeterministic) {
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-20.0f, 20.0f);
    std::vector<removert::Point> pts(2048);
    for (auto& p : pts) p = removert::Point{u(rng), u(rng), u(rng), 0.0f};

    removert::SensorConfig sensor; sensor.vfov_deg = 50.0f; sensor.hfov_deg = 360.0f;
    auto a = removert::project_to_range_image(pts, sensor, 1.0f, 4);
    auto b = removert::project_to_range_image(pts, sensor, 1.0f, 4);

    ASSERT_EQ(a.range.size(), b.range.size());
    for (std::size_t i = 0; i < a.range.size(); ++i)
        EXPECT_EQ(a.range[i], b.range[i]);
}
