// Synthetic end-to-end pipeline check.
//
// One static wall + one dynamic blob in front, two scans observing the wall.
// Expect: dynamic blob lands in `dynamic_map`, wall in `static_map`.

#include "removert/pipeline.hpp"
#include "removert/types.hpp"

#include <gtest/gtest.h>

using removert::PipelineConfig;
using removert::Point;
using removert::Pose;
using removert::remove_pipeline;
using removert::ScanFrame;

TEST(Pipeline, DynamicPointGetsRemoved) {
    std::vector<Point> map_world;
    map_world.push_back({4.0f, 0.0f, 0.0f, 0.0f});  // dynamic blob @ 4m
    for (int i = -5; i <= 5; ++i)
        map_world.push_back({10.0f, 0.05f * static_cast<float>(i), 0.0f, 0.0f});  // wall @ 10m

    auto make_scan = [](){
        ScanFrame f;
        for (int i = -5; i <= 5; ++i)
            f.points_local.push_back({10.0f, 0.05f * static_cast<float>(i), 0.0f, 0.0f});
        f.T_world_lidar = Pose{};
        return f;
    };
    std::vector<ScanFrame> scans = {make_scan(), make_scan(), make_scan()};

    PipelineConfig cfg{};
    cfg.sensor.vfov_deg = 6.0f; cfg.sensor.hfov_deg = 6.0f;
    cfg.remove.resolutions_deg = {0.5f};
    cfg.remove.diff_carve_m = 0.5f;
    cfg.remove.diff_hit_m   = 0.4f;
    cfg.remove.visibility.top_k          = 4;
    cfg.remove.visibility.log_odds_prior = 1.0f;
    cfg.remove.visibility.log_odds_carve = -1.5f;
    cfg.remove.visibility.log_odds_hit   = 1.0f;
    cfg.remove.visibility.decision_thr   = 0.0f;
    cfg.remove.visibility.log_odds_min   = -8.0f;
    cfg.remove.visibility.log_odds_max   = 8.0f;
    // Make revert non-flipping so the test focuses on Remove.
    cfg.revert.resolutions_deg = {};

    auto out = remove_pipeline(map_world, scans, cfg);
    EXPECT_EQ(out.dynamic_map.size(), 1u);
    EXPECT_GE(out.static_map .size(), 11u);
}
