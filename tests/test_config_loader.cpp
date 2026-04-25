#include "removert/io/config_loader.hpp"

#include <gtest/gtest.h>

#include <string>

using removert::io::dump_config_to_json;
using removert::io::load_config_from_json_string;

TEST(ConfigLoader, RoundTripDefault) {
    removert::PipelineConfig cfg{};
    auto s = dump_config_to_json(cfg);
    auto cfg2 = load_config_from_json_string(s);
    EXPECT_EQ(cfg2.remove.resolutions_deg.size(), cfg.remove.resolutions_deg.size());
    EXPECT_FLOAT_EQ(cfg2.remove.diff_hit_m, cfg.remove.diff_hit_m);
    EXPECT_EQ(cfg2.knn.k, cfg.knn.k);
    EXPECT_EQ(cfg2.start_idx, cfg.start_idx);
}

TEST(ConfigLoader, OverrideHyperparameters) {
    std::string const json = R"({
        "sensor": { "vfov_deg": 30.0, "hfov_deg": 180.0 },
        "remove": {
            "resolutions_deg": [2.0, 1.0],
            "diff_carve_m": 1.5,
            "visibility": { "top_k": 8, "log_odds_prior": 5.0 }
        },
        "knn": { "k": 12 },
        "start_idx": 100,
        "end_idx": 200
    })";
    auto cfg = load_config_from_json_string(json);
    EXPECT_FLOAT_EQ(cfg.sensor.vfov_deg, 30.0f);
    EXPECT_EQ(cfg.remove.resolutions_deg.size(), 2u);
    EXPECT_FLOAT_EQ(cfg.remove.diff_carve_m, 1.5f);
    EXPECT_EQ(cfg.remove.visibility.top_k, 8);
    EXPECT_FLOAT_EQ(cfg.remove.visibility.log_odds_prior, 5.0f);
    EXPECT_EQ(cfg.knn.k, 12);
    EXPECT_EQ(cfg.start_idx, 100);
    EXPECT_EQ(cfg.end_idx,   200);
}
