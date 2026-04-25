#include "removert/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace removert {

SphericalPoint cart_to_sph(Point p) noexcept {
    float const r  = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    float const az = std::atan2(p.y, p.x);
    float const el = (r > 0.0f) ? std::asin(std::clamp(p.z / r, -1.0f, 1.0f))
                                : 0.0f;
    return SphericalPoint{az, el, r};
}

Point sph_to_cart(SphericalPoint sp) noexcept {
    float const cos_el = std::cos(sp.el);
    return Point{sp.r * cos_el * std::cos(sp.az),
                 sp.r * cos_el * std::sin(sp.az),
                 sp.r * std::sin(sp.el),
                 0.0f};
}

Point transform(Point p, Eigen::Matrix4f const& T) noexcept {
    Eigen::Vector4f const v{p.x, p.y, p.z, 1.0f};
    Eigen::Vector4f const r = T * v;
    return Point{r.x(), r.y(), r.z(), p.intensity};
}

std::vector<Point> transform_all(std::vector<Point> const& pts,
                                 Eigen::Matrix4f const& T) {
    std::vector<Point> out;
    out.reserve(pts.size());
    std::transform(pts.begin(), pts.end(), std::back_inserter(out),
                   [&T](Point p) { return transform(p, T); });
    return out;
}

Eigen::Matrix4f se3_inverse(Eigen::Matrix4f const& T) noexcept {
    Eigen::Matrix4f inv = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f const R = T.block<3, 3>(0, 0);
    Eigen::Vector3f const t = T.block<3, 1>(0, 3);
    inv.block<3, 3>(0, 0) = R.transpose();
    inv.block<3, 1>(0, 3) = -R.transpose() * t;
    return inv;
}

}  // namespace removert
