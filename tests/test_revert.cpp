// Revert pass verification.
//
// Plan §P4: revert is the same accumulator as remove, with a stronger static
// prior. A point classified dynamic by remove can be pulled back to static if
// later scans show consistent matching hits.

#include "removert/geometry.hpp"
#include "removert/range_image.hpp"
#include "removert/revert.hpp"
#include "removert/types.hpp"

#include <gtest/gtest.h>

using removert::accumulate_visibility;
using removert::classify_reverted;
using removert::make_revert_state;
using removert::Point;
using removert::Pose;
using removert::project_to_range_image;
using removert::RemoveConfig;
using removert::revert_pass;
using removert::RevertConfig;
using removert::ScanFrame;
using removert::SensorConfig;

TEST(Revert, MatchingHitsKeepCandidatesStatic) {
    SensorConfig sensor; sensor.vfov_deg = 4.0f; sensor.hfov_deg = 4.0f;

    // Candidate is at 5m, scans see it at 5m -> matching hit.
    std::vector<Point> candidates{{5.0f, 0.0f, 0.0f, 0.0f}};
    ScanFrame scan;
    scan.points_local   = {{5.0f, 0.0f, 0.0f, 0.0f}};
    scan.T_world_lidar  = Pose{};

    RevertConfig cfg{};
    cfg.diff_carve_m = 0.5f;
    cfg.diff_hit_m   = 0.4f;
    cfg.visibility.top_k          = 1;
    cfg.visibility.log_odds_prior = 0.5f;
    cfg.visibility.log_odds_carve = -0.2f;
    cfg.visibility.log_odds_hit   = 1.0f;
    cfg.visibility.decision_thr   = 0.0f;
    cfg.visibility.log_odds_min   = -8.0f;
    cfg.visibility.log_odds_max   = 8.0f;

    auto state = make_revert_state(candidates.size(), cfg);
    revert_pass(candidates, scan, sensor, cfg, /*resolution_deg=*/1.0f, state);
    EXPECT_GT(state.log_odds[0], 0.5f);

    auto kept = classify_reverted(state, cfg);
    ASSERT_EQ(kept.size(), 1u);
    EXPECT_EQ(kept[0], 0);
}

TEST(Revert, CarvingPushesBelowThreshold) {
    SensorConfig sensor; sensor.vfov_deg = 4.0f; sensor.hfov_deg = 4.0f;

    // Candidate at 5m, scan returns at 10m -> carving evidence.
    std::vector<Point> candidates{{5.0f, 0.0f, 0.0f, 0.0f}};
    ScanFrame scan;
    scan.points_local   = {{10.0f, 0.0f, 0.0f, 0.0f}};
    scan.T_world_lidar  = Pose{};

    RevertConfig cfg{};
    cfg.diff_carve_m = 0.5f;
    cfg.visibility.top_k          = 1;
    cfg.visibility.log_odds_prior = 0.2f;
    cfg.visibility.log_odds_carve = -1.0f;
    cfg.visibility.log_odds_hit   = 1.0f;
    cfg.visibility.decision_thr   = 0.0f;
    cfg.visibility.log_odds_min   = -8.0f;
    cfg.visibility.log_odds_max   = 8.0f;

    auto state = make_revert_state(candidates.size(), cfg);
    revert_pass(candidates, scan, sensor, cfg, /*resolution_deg=*/1.0f, state);

    auto kept = classify_reverted(state, cfg);
    EXPECT_TRUE(kept.empty());
}
