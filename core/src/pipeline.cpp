#include "removert/pipeline.hpp"

#include "removert/remove.hpp"
#include "removert/revert.hpp"

#include <algorithm>
#include <unordered_set>

namespace removert {

namespace {

std::pair<std::vector<Point>, std::vector<Point>>
split_by_indices(std::vector<Point> const& src,
                 std::vector<int> const& dynamic_idx) {
    std::unordered_set<int> dyn(dynamic_idx.begin(), dynamic_idx.end());
    std::vector<Point> stat, dynp;
    stat.reserve(src.size() - dyn.size());
    dynp.reserve(dyn.size());
    for (int i = 0; i < static_cast<int>(src.size()); ++i) {
        if (dyn.count(i) != 0u) dynp.push_back(src[static_cast<std::size_t>(i)]);
        else                    stat.push_back(src[static_cast<std::size_t>(i)]);
    }
    return {std::move(stat), std::move(dynp)};
}

void run_remove(std::vector<Point> const& map_global,
                std::vector<ScanFrame> const& scans,
                PipelineConfig const& cfg,
                VisibilityState& state) {
    int const begin = std::max(0, cfg.start_idx);
    int const end   = (cfg.end_idx >= 0)
                          ? std::min<int>(cfg.end_idx, static_cast<int>(scans.size()))
                          : static_cast<int>(scans.size());
    int const gap = std::max(1, cfg.keyframe_gap);

    for (float res_deg : cfg.remove.resolutions_deg) {
        for (int i = begin; i < end; i += gap) {
            remove_pass(map_global, scans[static_cast<std::size_t>(i)],
                        cfg.sensor, cfg.remove, res_deg, state);
        }
    }
}

void run_revert(std::vector<Point> const& dynamic_candidates,
                std::vector<ScanFrame> const& scans,
                PipelineConfig const& cfg,
                VisibilityState& state) {
    int const begin = std::max(0, cfg.start_idx);
    int const end   = (cfg.end_idx >= 0)
                          ? std::min<int>(cfg.end_idx, static_cast<int>(scans.size()))
                          : static_cast<int>(scans.size());
    int const gap = std::max(1, cfg.keyframe_gap);

    for (float res_deg : cfg.revert.resolutions_deg) {
        for (int i = begin; i < end; i += gap) {
            revert_pass(dynamic_candidates,
                        scans[static_cast<std::size_t>(i)],
                        cfg.sensor, cfg.revert, res_deg, state);
        }
    }
}

}  // namespace

PipelineOutput remove_pipeline(std::vector<Point> map_global,
                               std::vector<ScanFrame> const& scans,
                               PipelineConfig const& cfg) {
    VisibilityState state =
        make_visibility_state(map_global.size(), cfg.remove.visibility);
    run_remove(map_global, scans, cfg, state);

    std::vector<int> dyn_idx = classify_dynamic(state, cfg.remove.visibility);
    auto [static_pts, dynamic_pts] = split_by_indices(map_global, dyn_idx);

    // Revert phase: skip entirely when no revert resolutions are configured —
    // otherwise classify_reverted on the un-touched (prior-only) state would
    // mark every candidate static again and cancel Remove.
    std::vector<int> reverted;
    if (!cfg.revert.resolutions_deg.empty()) {
        VisibilityState rstate =
            make_revert_state(dynamic_pts.size(), cfg.revert);
        run_revert(dynamic_pts, scans, cfg, rstate);
        reverted = classify_reverted(rstate, cfg.revert);
    }

    if (!reverted.empty()) {
        std::unordered_set<int> rev(reverted.begin(), reverted.end());
        std::vector<Point> still_dyn;
        still_dyn.reserve(dynamic_pts.size() - rev.size());
        for (int i = 0; i < static_cast<int>(dynamic_pts.size()); ++i) {
            if (rev.count(i) != 0u)
                static_pts.push_back(dynamic_pts[static_cast<std::size_t>(i)]);
            else
                still_dyn.push_back(dynamic_pts[static_cast<std::size_t>(i)]);
        }
        dynamic_pts = std::move(still_dyn);
    }

    return PipelineOutput{std::move(static_pts), std::move(dynamic_pts),
                          std::move(state.log_odds)};
}

}  // namespace removert
