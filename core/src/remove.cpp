#include "removert/remove.hpp"

#include "removert/geometry.hpp"
#include "removert/range_image.hpp"

#include <algorithm>
#include <cassert>

namespace removert {

namespace {

// Apply per-map-point evidence to the log-odds buffer with clamping.
inline void apply_evidence(VisibilityState& state, int idx, float delta,
                           float lo_min, float lo_max) noexcept {
    if (idx < 0 || idx >= static_cast<int>(state.log_odds.size())) return;
    float v = state.log_odds[static_cast<std::size_t>(idx)] + delta;
    state.log_odds[static_cast<std::size_t>(idx)] =
        std::clamp(v, lo_min, lo_max);
}

}  // namespace

VisibilityState make_visibility_state(std::size_t num_map_points,
                                      VisibilityConfig const& vis) {
    return VisibilityState{std::vector<float>(num_map_points, vis.log_odds_prior)};
}

// For each pixel, compare scan range I_q (slot 0) against the K map slots.
//   * Slots whose range is "behind" the scan return  -> hit evidence.
//   * Slots whose range is "between origin and the scan return", with the gap
//     exceeding diff_carve_m -> carve evidence.
// The same accumulation rule is shared by Remove and Revert; only the sign /
// magnitude of the evidence differs (held in cfg.visibility).
void accumulate_visibility(RangeImage const& scan_rimg,
                           RangeImage const& map_rimg,
                           RemoveConfig const& cfg,
                           VisibilityState& state) {
    assert(scan_rimg.rows == map_rimg.rows);
    assert(scan_rimg.cols == map_rimg.cols);

    VisibilityConfig const& v = cfg.visibility;
    int const rows = scan_rimg.rows;
    int const cols = scan_rimg.cols;
    int const k    = map_rimg.top_k;

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            std::size_t const scan_base =
                (static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) +
                 static_cast<std::size_t>(c)) *
                static_cast<std::size_t>(scan_rimg.top_k);
            float const scan_r = scan_rimg.range[scan_base];
            if (!std::isfinite(scan_r)) continue;  // no scan return for this pixel

            std::size_t const map_base =
                (static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) +
                 static_cast<std::size_t>(c)) * static_cast<std::size_t>(k);
            for (int s = 0; s < k; ++s) {
                float const mr = map_rimg.range[map_base + static_cast<std::size_t>(s)];
                if (!std::isfinite(mr)) break;  // sorted ascending; rest empty
                int   const mi = map_rimg.src_idx[map_base + static_cast<std::size_t>(s)];
                float const diff = scan_r - mr;  // >0 -> map is closer than scan
                if (diff > cfg.diff_carve_m) {
                    apply_evidence(state, mi, v.log_odds_carve,
                                   v.log_odds_min, v.log_odds_max);
                } else if (std::abs(diff) <= cfg.diff_hit_m) {
                    apply_evidence(state, mi, v.log_odds_hit,
                                   v.log_odds_min, v.log_odds_max);
                }
                // else: scan return is closer than map (occluded). No evidence.
            }
        }
    }
}

std::vector<int> classify_dynamic(VisibilityState const& state,
                                  VisibilityConfig const& vis) {
    std::vector<int> out;
    out.reserve(state.log_odds.size() / 32);
    for (std::size_t i = 0; i < state.log_odds.size(); ++i) {
        if (state.log_odds[i] < vis.decision_thr) {
            out.push_back(static_cast<int>(i));
        }
    }
    return out;
}

void remove_pass(std::vector<Point> const& map_global,
                 ScanFrame const& scan,
                 SensorConfig const& sensor,
                 RemoveConfig const& cfg,
                 float resolution_deg,
                 VisibilityState& state) {
    Eigen::Matrix4f const T_lidar_world = se3_inverse(scan.T_world_lidar.T);
    std::vector<Point> map_in_sensor =
        transform_all(map_global, T_lidar_world);

    RangeImage map_rimg = project_to_range_image(
        map_in_sensor, sensor, resolution_deg, cfg.visibility.top_k);
    RangeImage scan_rimg = project_to_range_image(
        scan.points_local, sensor, resolution_deg, /*top_k=*/1);

    accumulate_visibility(scan_rimg, map_rimg, cfg, state);
}

}  // namespace removert
