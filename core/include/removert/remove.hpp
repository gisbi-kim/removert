// Strengthened-visibility "remove" pass.
//
// Plan §4.8: combine multi-hit (A1), free-segment ray carving (A2), and
// log-odds time accumulation (A3) into one unified visibility pipeline.
//
// All functions are pure: no I/O, no member state.

#pragma once

#include "removert/types.hpp"

#include <cstdint>
#include <vector>

namespace removert {

// Per-map-point evidence accumulator. One entry per map point. We track
// log-odds on the static-vs-dynamic axis: positive => static, negative =>
// dynamic. The accumulator is *not* a function of any particular scan and is
// the only piece of mutable state passed between scans.
struct VisibilityState {
    std::vector<float> log_odds;
};

[[nodiscard]] VisibilityState
make_visibility_state(std::size_t num_map_points,
                      VisibilityConfig const& vis);

// Update visibility evidence by comparing one scan's range image to the map's
// range image at the same resolution. Modifies state.log_odds.
//
// Inputs:
//   scan_rimg : top-1 (or top-K) projection of the scan as seen from the
//               sensor at this scan's pose. src_idx is into the *scan*.
//   map_rimg  : top-K projection of the map points (in sensor frame for this
//               scan). src_idx is into the *map*.
//
// The two range images must share rows / cols / top_k.
//
// `cfg.diff_carve_m` and `cfg.diff_hit_m` define the evidence rules. See
// docs/v2-plan.md §4.8 for the algorithm.
void accumulate_visibility(RangeImage const& scan_rimg,
                           RangeImage const& map_rimg,
                           RemoveConfig const& cfg,
                           VisibilityState& state);

// After accumulation, points whose log-odds is below `vis.decision_thr` are
// classified dynamic. Returns the indices of dynamic points in the *map*.
[[nodiscard]] std::vector<int>
classify_dynamic(VisibilityState const& state, VisibilityConfig const& vis);

// One full Remove iteration at one resolution. Builds the map's range image
// in the scan's sensor frame, builds the scan's range image, then accumulates
// evidence. Returns nothing — state is updated in place.
void remove_pass(std::vector<Point> const& map_global,
                 ScanFrame const& scan,
                 SensorConfig const& sensor,
                 RemoveConfig const& cfg,
                 float resolution_deg,
                 VisibilityState& state);

}  // namespace removert
