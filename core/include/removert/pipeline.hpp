// Top-level compose: feed all (or a subset of) scans through Remove + Revert
// at multiple resolutions, returning a static / dynamic split of the map.
//
// Pure: takes the global map by value (move-friendly) and returns owned
// vectors. The caller manages I/O.

#pragma once

#include "removert/types.hpp"

#include <vector>

namespace removert {

struct PipelineOutput {
    std::vector<Point> static_map;
    std::vector<Point> dynamic_map;
    // Per-point final log-odds, aligned with the *original* map_global order.
    std::vector<float> final_log_odds;
};

[[nodiscard]] PipelineOutput
remove_pipeline(std::vector<Point> map_global,
                std::vector<ScanFrame> const& scans,
                PipelineConfig const& cfg);

}  // namespace removert
