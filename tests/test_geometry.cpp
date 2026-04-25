#include "removert/geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>

using removert::cart_to_sph;
using removert::Point;
using removert::se3_inverse;
using removert::sph_to_cart;
using removert::transform;
using removert::transform_all;

TEST(Geometry, CartToSphAxisX) {
    auto sp = cart_to_sph(Point{1.0f, 0.0f, 0.0f, 0.0f});
    EXPECT_NEAR(sp.r, 1.0f, 1e-6);
    EXPECT_NEAR(sp.az, 0.0f, 1e-6);
    EXPECT_NEAR(sp.el, 0.0f, 1e-6);
}

TEST(Geometry, RoundTripSphCart) {
    Point p{2.0f, -3.0f, 0.5f, 7.0f};
    auto sp = cart_to_sph(p);
    auto q  = sph_to_cart(sp);
    EXPECT_NEAR(q.x, p.x, 1e-5);
    EXPECT_NEAR(q.y, p.y, 1e-5);
    EXPECT_NEAR(q.z, p.z, 1e-5);
}

TEST(Geometry, TransformIdentityIsNoOp) {
    Point p{1.0f, 2.0f, 3.0f, 4.0f};
    auto q = transform(p, Eigen::Matrix4f::Identity());
    EXPECT_EQ(q.x, p.x);
    EXPECT_EQ(q.y, p.y);
    EXPECT_EQ(q.z, p.z);
    EXPECT_EQ(q.intensity, p.intensity);
}

TEST(Geometry, TransformTranslation) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0, 3) = 5.0f;
    T(1, 3) = -2.0f;
    auto q = transform(Point{1, 1, 1, 0}, T);
    EXPECT_FLOAT_EQ(q.x, 6.0f);
    EXPECT_FLOAT_EQ(q.y, -1.0f);
    EXPECT_FLOAT_EQ(q.z, 1.0f);
}

TEST(Geometry, TransformAllPreservesSize) {
    std::vector<Point> pts{{0, 0, 0, 0}, {1, 1, 1, 0}, {2, 2, 2, 0}};
    auto out = transform_all(pts, Eigen::Matrix4f::Identity());
    ASSERT_EQ(out.size(), pts.size());
}

TEST(Geometry, Se3InverseIsLeftRightInverse) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0, 3) = 1.0f; T(1, 3) = 2.0f; T(2, 3) = 3.0f;
    // A 30-deg rotation around z.
    float const c = std::cos(0.5f), s = std::sin(0.5f);
    T(0, 0) = c; T(0, 1) = -s;
    T(1, 0) = s; T(1, 1) =  c;

    Eigen::Matrix4f inv = se3_inverse(T);
    Eigen::Matrix4f I   = T * inv;
    for (int r = 0; r < 4; ++r)
        for (int c2 = 0; c2 < 4; ++c2)
            EXPECT_NEAR(I(r, c2), (r == c2 ? 1.0f : 0.0f), 1e-5f);
}
