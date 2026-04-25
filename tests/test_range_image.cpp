#include "removert/range_image.hpp"
#include "removert/geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using removert::Point;
using removert::project_to_range_image;
using removert::range_image_dims;
using removert::SensorConfig;
using removert::sph_to_pixel;
using removert::cart_to_sph;

TEST(RangeImage, DimensionsAreCeiled) {
    SensorConfig s; s.vfov_deg = 50.0f; s.hfov_deg = 360.0f;
    auto [r, c] = range_image_dims(s, 1.0f);
    EXPECT_EQ(r, 50);
    EXPECT_EQ(c, 360);
}

TEST(RangeImage, ProjectingFrontPointHitsCenterCol) {
    SensorConfig s; s.vfov_deg = 50.0f; s.hfov_deg = 360.0f;
    Point p{1.0f, 0.0f, 0.0f, 0.0f};
    auto sp = cart_to_sph(p);
    auto [r, c] = sph_to_pixel(sp, s, 1.0f, 50, 360);
    EXPECT_EQ(c, 180);  // 0 deg azimuth -> middle column
}

TEST(RangeImage, EmptyInputIsAllNoRange) {
    SensorConfig s;
    auto rimg = project_to_range_image({}, s, 1.0f, 1);
    for (auto v : rimg.range) EXPECT_TRUE(std::isinf(v));
}

TEST(RangeImage, TopKSlotsSortAscending) {
    SensorConfig s;
    // Three points at the same azimuth/elevation, different ranges.
    std::vector<Point> pts{
        {3.0f, 0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f, 0.0f},
        {2.0f, 0.0f, 0.0f, 0.0f},
    };
    auto rimg = project_to_range_image(pts, s, 1.0f, /*top_k=*/3);
    auto [r, c] = sph_to_pixel(cart_to_sph(pts[0]), s, 1.0f, rimg.rows, rimg.cols);
    ASSERT_GE(r, 0);
    auto base = static_cast<std::size_t>(r * rimg.cols + c) * 3;
    EXPECT_NEAR(rimg.range[base + 0], 1.0f, 1e-5);
    EXPECT_NEAR(rimg.range[base + 1], 2.0f, 1e-5);
    EXPECT_NEAR(rimg.range[base + 2], 3.0f, 1e-5);
}

TEST(RangeImage, OutOfFovIsRejected) {
    SensorConfig s; s.vfov_deg = 10.0f; s.hfov_deg = 10.0f;
    Point p{0.0f, 1.0f, 0.0f, 0.0f};  // 90deg azimuth — outside ±5 deg
    auto sp = cart_to_sph(p);
    auto [r, c] = sph_to_pixel(sp, s, 1.0f, 10, 10);
    EXPECT_EQ(r, -1);
    EXPECT_EQ(c, -1);
}
