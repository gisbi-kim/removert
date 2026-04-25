#include "removert/range_image.hpp"

#include "removert/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace removert {

namespace {

constexpr float kDeg2Rad = 0.017453292519943295f;

std::pair<int, int> compute_dims(float vfov_deg, float hfov_deg,
                                 float res_deg) noexcept {
    int const rows = std::max(1, static_cast<int>(std::ceil(vfov_deg / res_deg)));
    int const cols = std::max(1, static_cast<int>(std::ceil(hfov_deg / res_deg)));
    return {rows, cols};
}

// Insert (range, src) into a sorted top-K slot block. Slots are kept ascending.
void insert_top_k(float* range_slot, int* idx_slot, int top_k,
                  float r, int src) noexcept {
    if (r >= range_slot[top_k - 1]) return;
    int pos = top_k - 1;
    while (pos > 0 && range_slot[pos - 1] > r) {
        range_slot[pos] = range_slot[pos - 1];
        idx_slot  [pos] = idx_slot  [pos - 1];
        --pos;
    }
    range_slot[pos] = r;
    idx_slot  [pos] = src;
}

}  // namespace

std::pair<int, int>
range_image_dims(SensorConfig const& sensor, float resolution_deg) noexcept {
    return compute_dims(sensor.vfov_deg, sensor.hfov_deg, resolution_deg);
}

std::pair<int, int>
sph_to_pixel(SphericalPoint sp,
             SensorConfig const& sensor,
             float resolution_deg,
             int rows, int cols) noexcept {
    float const az_deg = sp.az / kDeg2Rad;
    float const el_deg = sp.el / kDeg2Rad;
    float const az_min = -sensor.hfov_deg * 0.5f;
    float const el_max =  sensor.vfov_deg * 0.5f;

    if (az_deg < az_min || az_deg >= az_min + sensor.hfov_deg) return {-1, -1};
    if (el_deg < el_max - sensor.vfov_deg || el_deg > el_max)  return {-1, -1};

    int const r = static_cast<int>(std::floor((el_max - el_deg) / resolution_deg));
    int const c = static_cast<int>(std::floor((az_deg - az_min) / resolution_deg));
    if (r < 0 || r >= rows) return {-1, -1};
    if (c < 0 || c >= cols) return {-1, -1};
    return {r, c};
}

RangeImage project_to_range_image(std::vector<Point> const& pts,
                                  SensorConfig const& sensor,
                                  float resolution_deg,
                                  int top_k) {
    RangeImage img;
    auto [rows, cols] = range_image_dims(sensor, resolution_deg);
    img.rows  = rows;
    img.cols  = cols;
    img.top_k = std::max(1, top_k);
    auto const slots = static_cast<std::size_t>(rows) *
                       static_cast<std::size_t>(cols) *
                       static_cast<std::size_t>(img.top_k);
    img.range  .assign(slots, kNoRange);
    img.src_idx.assign(slots, kNoIndex);

    int const k = img.top_k;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        SphericalPoint const sp = cart_to_sph(pts[i]);
        if (sp.r <= 0.0f) continue;
        auto [r, c] = sph_to_pixel(sp, sensor, resolution_deg, rows, cols);
        if (r < 0) continue;
        std::size_t const base =
            (static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) +
             static_cast<std::size_t>(c)) * static_cast<std::size_t>(k);
        insert_top_k(&img.range[base], &img.src_idx[base], k,
                     sp.r, static_cast<int>(i));
    }
    return img;
}

}  // namespace removert
