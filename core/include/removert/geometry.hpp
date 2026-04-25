// Geometry primitives — pure functions, no state.
#pragma once

#include "removert/types.hpp"

#include <Eigen/Core>
#include <vector>

namespace removert {

[[nodiscard]] SphericalPoint cart_to_sph(Point p) noexcept;
[[nodiscard]] Point          sph_to_cart(SphericalPoint sp) noexcept;
[[nodiscard]] Point          transform(Point p, Eigen::Matrix4f const& T) noexcept;

[[nodiscard]] std::vector<Point>
transform_all(std::vector<Point> const& pts, Eigen::Matrix4f const& T);

// SE3 inverse for float Eigen matrices (no Eigen::Affine dependency).
[[nodiscard]] Eigen::Matrix4f se3_inverse(Eigen::Matrix4f const& T) noexcept;

}  // namespace removert
