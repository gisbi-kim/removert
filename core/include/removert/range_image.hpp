// Range-image projection — multi-hit (top-K) per pixel.  See plan §4.8 (A1).
#pragma once

#include "removert/types.hpp"

#include <utility>
#include <vector>

namespace removert {

// Returns (rows, cols) for a given resolution and FOV. rows = vfov / res,
// cols = hfov / res, ceiled, clamped to >= 1.
[[nodiscard]] std::pair<int, int>
range_image_dims(SensorConfig const& sensor, float resolution_deg) noexcept;

// Map a spherical point to (row, col). Returns (-1, -1) if out of FOV.
// Uses the convention:
//   row = (elevation_max - el_deg) / res     (top row = highest elevation)
//   col = (az_deg - az_min) / res            (col=0 is at -hfov/2)
// The angle range is centred on (0, 0).
[[nodiscard]] std::pair<int, int>
sph_to_pixel(SphericalPoint sp,
             SensorConfig const& sensor,
             float resolution_deg,
             int rows, int cols) noexcept;

// Build a top-K range image. `pts` are assumed to live in the sensor frame
// (i.e. caller transformed them already).
[[nodiscard]] RangeImage
project_to_range_image(std::vector<Point> const& pts,
                       SensorConfig const& sensor,
                       float resolution_deg,
                       int top_k);

// Single-hit convenience overload (top_k = 1).
[[nodiscard]] inline RangeImage
project_to_range_image(std::vector<Point> const& pts,
                       SensorConfig const& sensor,
                       float resolution_deg) {
    return project_to_range_image(pts, sensor, resolution_deg, 1);
}

}  // namespace removert
