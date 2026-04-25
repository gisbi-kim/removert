// Revert pass — log-odds prior tweak (plan §P4).
//
// Revert is the same accumulation algorithm as remove, but starting from a
// stronger static prior over the *static-candidate* set, so points that were
// erroneously classified dynamic during Remove (e.g. transient occluders)
// flip back to static.
//
// We keep the function separate for readability; it shares accumulate_visibility
// internally.

#pragma once

#include "removert/types.hpp"
#include "removert/remove.hpp"

#include <vector>

namespace removert {

// Build a fresh visibility state for the revert phase using `cfg.visibility`'s
// (typically stronger) static prior.
[[nodiscard]] VisibilityState
make_revert_state(std::size_t num_static_candidates, RevertConfig const& cfg);

// Run one revert iteration at one resolution. Updates `state.log_odds`. The
// caller passes the *static-candidate* point cloud — typically the points
// previously classified dynamic by Remove (so revert can pull them back).
void revert_pass(std::vector<Point> const& candidates_global,
                 ScanFrame const& scan,
                 SensorConfig const& sensor,
                 RevertConfig const& cfg,
                 float resolution_deg,
                 VisibilityState& state);

// After accumulation, points whose log-odds is *above* the decision threshold
// are reverted back to static. Returns indices into `candidates_global`.
[[nodiscard]] std::vector<int>
classify_reverted(VisibilityState const& state, RevertConfig const& cfg);

}  // namespace removert
