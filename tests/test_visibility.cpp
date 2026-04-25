// Synthetic-scene unit test for the strengthened visibility model.
// Plan §A1+A2+A3: build a scene with a known dynamic point in front of a
// static wall, run remove_pass, and check that the dynamic point's log-odds
// drops below the static prior while wall points stay above.

#include "removert/geometry.hpp"
#include "removert/range_image.hpp"
#include "removert/remove.hpp"
#include "removert/types.hpp"

#include <gtest/gtest.h>

#include <vector>

using removert::Point;
using removert::Pose;
using removert::ScanFrame;
using removert::SensorConfig;
using removert::RemoveConfig;
using removert::VisibilityConfig;
using removert::accumulate_visibility;
using removert::classify_dynamic;
using removert::make_visibility_state;
using removert::project_to_range_image;
using removert::remove_pass;
using removert::transform_all;

namespace {

ScanFrame make_scan_at_origin(std::vector<Point> pts) {
    ScanFrame f;
    f.points_local   = std::move(pts);
    f.T_world_lidar  = Pose{};
    f.stamp_ns       = 0;
    return f;
}

}  // namespace

TEST(Visibility, MakeStateInitializesPrior) {
    VisibilityConfig vis; vis.log_odds_prior = 1.5f;
    auto st = make_visibility_state(10, vis);
    ASSERT_EQ(st.log_odds.size(), 10u);
    for (auto v : st.log_odds) EXPECT_FLOAT_EQ(v, 1.5f);
}

TEST(Visibility, CarvingDropsLogOddsBelowPrior) {
    // Map: point at 5m straight ahead.
    // Scan: return at 10m straight ahead — therefore the 5m map point should
    // be carved (free space evidence).
    SensorConfig sensor; sensor.vfov_deg = 4.0f; sensor.hfov_deg = 4.0f;

    std::vector<Point> map_pts{{5.0f, 0.0f, 0.0f, 0.0f}};
    std::vector<Point> scan_pts{{10.0f, 0.0f, 0.0f, 0.0f}};

    RemoveConfig cfg{};
    cfg.diff_carve_m = 0.5f;
    cfg.diff_hit_m   = 0.4f;
    cfg.visibility.top_k          = 1;
    cfg.visibility.log_odds_prior = 1.0f;
    cfg.visibility.log_odds_carve = -2.0f;
    cfg.visibility.log_odds_hit   =  1.0f;
    cfg.visibility.decision_thr   = 0.0f;
    cfg.visibility.log_odds_min   = -8.0f;
    cfg.visibility.log_odds_max   =  8.0f;

    auto state = make_visibility_state(map_pts.size(), cfg.visibility);

    auto map_rimg  = project_to_range_image(map_pts,  sensor, 1.0f, 1);
    auto scan_rimg = project_to_range_image(scan_pts, sensor, 1.0f, 1);
    accumulate_visibility(scan_rimg, map_rimg, cfg, state);

    EXPECT_LT(state.log_odds[0], 1.0f) << "carving must reduce log-odds";
    auto dyn = classify_dynamic(state, cfg.visibility);
    EXPECT_EQ(dyn.size(), 1u);
}

TEST(Visibility, MatchingHitRaisesLogOdds) {
    SensorConfig sensor; sensor.vfov_deg = 4.0f; sensor.hfov_deg = 4.0f;
    std::vector<Point> map_pts {{5.0f, 0.0f, 0.0f, 0.0f}};
    std::vector<Point> scan_pts{{5.0f, 0.0f, 0.0f, 0.0f}};

    RemoveConfig cfg{};
    cfg.diff_hit_m = 0.4f;
    cfg.visibility.log_odds_prior = 0.0f;
    cfg.visibility.log_odds_hit   = 1.0f;
    cfg.visibility.log_odds_carve = -1.0f;
    cfg.visibility.log_odds_max   = 8.0f;
    cfg.visibility.log_odds_min   = -8.0f;

    auto state = make_visibility_state(1, cfg.visibility);
    auto map_r  = project_to_range_image(map_pts,  sensor, 1.0f, 1);
    auto scan_r = project_to_range_image(scan_pts, sensor, 1.0f, 1);
    accumulate_visibility(scan_r, map_r, cfg, state);

    EXPECT_GT(state.log_odds[0], 0.0f);
}

TEST(Visibility, RemovePassEndToEnd) {
    // Map (in world frame): one dynamic point in front of a small wall.
    //   index 0   : dynamic, at  3m
    //   indices 1+: wall at      8m, at small lateral offsets
    // Scan returns 8m wall returns -> evidence carves index 0.
    std::vector<Point> map_world{
        {3.0f, 0.0f, 0.0f, 0.0f},
        {8.0f, 0.0f, 0.0f, 0.0f},
        {8.0f, 0.05f, 0.0f, 0.0f},
        {8.0f, -0.05f, 0.0f, 0.0f},
    };
    std::vector<Point> scan_local{
        {8.0f, 0.0f, 0.0f, 0.0f},
        {8.0f, 0.05f, 0.0f, 0.0f},
        {8.0f, -0.05f, 0.0f, 0.0f},
    };
    auto scan = make_scan_at_origin(scan_local);

    RemoveConfig cfg{};
    cfg.visibility.top_k          = 4;
    cfg.visibility.log_odds_prior = 1.0f;
    cfg.visibility.log_odds_carve = -2.0f;
    cfg.visibility.log_odds_hit   =  1.0f;
    cfg.visibility.decision_thr   = 0.0f;
    cfg.visibility.log_odds_min   = -8.0f;
    cfg.visibility.log_odds_max   =  8.0f;
    cfg.diff_carve_m = 0.5f;
    cfg.diff_hit_m   = 0.4f;

    SensorConfig sensor; sensor.vfov_deg = 4.0f; sensor.hfov_deg = 4.0f;
    auto state = make_visibility_state(map_world.size(), cfg.visibility);
    remove_pass(map_world, scan, sensor, cfg, /*resolution_deg=*/0.5f, state);

    auto dyn = classify_dynamic(state, cfg.visibility);
    ASSERT_EQ(dyn.size(), 1u);
    EXPECT_EQ(dyn[0], 0);
}
