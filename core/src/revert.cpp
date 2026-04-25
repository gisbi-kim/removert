#include "removert/revert.hpp"

#include "removert/geometry.hpp"
#include "removert/range_image.hpp"

#include <algorithm>

namespace removert {

VisibilityState make_revert_state(std::size_t num_static_candidates,
                                  RevertConfig const& cfg) {
    return VisibilityState{
        std::vector<float>(num_static_candidates, cfg.visibility.log_odds_prior)};
}

void revert_pass(std::vector<Point> const& candidates_global,
                 ScanFrame const& scan,
                 SensorConfig const& sensor,
                 RevertConfig const& cfg,
                 float resolution_deg,
                 VisibilityState& state) {
    // Reuse the Remove accumulator by lifting RevertConfig into a transient
    // RemoveConfig view. (Same math, different evidence weights.)
    RemoveConfig adapter{};
    adapter.diff_carve_m = cfg.diff_carve_m;
    adapter.diff_hit_m   = cfg.diff_hit_m;
    adapter.visibility   = cfg.visibility;

    Eigen::Matrix4f const T_lidar_world = se3_inverse(scan.T_world_lidar.T);
    std::vector<Point> cands_in_sensor =
        transform_all(candidates_global, T_lidar_world);

    RangeImage cand_rimg = project_to_range_image(
        cands_in_sensor, sensor, resolution_deg, cfg.visibility.top_k);
    RangeImage scan_rimg = project_to_range_image(
        scan.points_local, sensor, resolution_deg, /*top_k=*/1);

    accumulate_visibility(scan_rimg, cand_rimg, adapter, state);
}

std::vector<int> classify_reverted(VisibilityState const& state,
                                   RevertConfig const& cfg) {
    std::vector<int> out;
    out.reserve(state.log_odds.size() / 32);
    for (std::size_t i = 0; i < state.log_odds.size(); ++i) {
        if (state.log_odds[i] >= cfg.visibility.decision_thr) {
            out.push_back(static_cast<int>(i));
        }
    }
    return out;
}

}  // namespace removert
