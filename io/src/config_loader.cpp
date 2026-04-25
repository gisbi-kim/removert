#include "removert/io/config_loader.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>

namespace removert::io {

namespace {

using nlohmann::json;

template <typename T>
T pick(json const& j, char const* key, T fallback) {
    if (!j.contains(key)) return fallback;
    return j.at(key).get<T>();
}

void load_visibility(json const& j, VisibilityConfig& v) {
    v.top_k          = pick(j, "top_k",          v.top_k);
    v.log_odds_prior = pick(j, "log_odds_prior", v.log_odds_prior);
    v.log_odds_carve = pick(j, "log_odds_carve", v.log_odds_carve);
    v.log_odds_hit   = pick(j, "log_odds_hit",   v.log_odds_hit);
    v.decision_thr   = pick(j, "decision_thr",   v.decision_thr);
    v.log_odds_min   = pick(j, "log_odds_min",   v.log_odds_min);
    v.log_odds_max   = pick(j, "log_odds_max",   v.log_odds_max);
}

void load_sensor(json const& j, SensorConfig& s) {
    s.vfov_deg = pick(j, "vfov_deg", s.vfov_deg);
    s.hfov_deg = pick(j, "hfov_deg", s.hfov_deg);
    if (j.contains("T_pose_lidar")) {
        auto const& a = j.at("T_pose_lidar");
        if (a.is_array() && a.size() == 16) {
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    s.T_pose_lidar(r, c) = a.at(static_cast<std::size_t>(r * 4 + c))
                                              .get<float>();
        }
    }
}

void load_remove(json const& j, RemoveConfig& r) {
    r.resolutions_deg = pick(j, "resolutions_deg", r.resolutions_deg);
    r.diff_carve_m    = pick(j, "diff_carve_m",    r.diff_carve_m);
    r.diff_hit_m      = pick(j, "diff_hit_m",      r.diff_hit_m);
    if (j.contains("visibility")) load_visibility(j.at("visibility"), r.visibility);
}

void load_revert(json const& j, RevertConfig& r) {
    r.resolutions_deg = pick(j, "resolutions_deg", r.resolutions_deg);
    r.diff_carve_m    = pick(j, "diff_carve_m",    r.diff_carve_m);
    r.diff_hit_m      = pick(j, "diff_hit_m",      r.diff_hit_m);
    if (j.contains("visibility")) load_visibility(j.at("visibility"), r.visibility);
}

PipelineConfig from_json(json const& root) {
    PipelineConfig cfg{};
    if (root.contains("sensor"))     load_sensor(root.at("sensor"),     cfg.sensor);
    if (root.contains("remove"))     load_remove(root.at("remove"),     cfg.remove);
    if (root.contains("revert"))     load_revert(root.at("revert"),     cfg.revert);
    if (root.contains("knn")) {
        auto const& jk = root.at("knn");
        cfg.knn.k                    = pick(jk, "k",                  cfg.knn.k);
        cfg.knn.radius_m             = pick(jk, "radius_m",           cfg.knn.radius_m);
        cfg.knn.scan_map_avg_diff_thr =
            pick(jk, "scan_map_avg_diff_thr", cfg.knn.scan_map_avg_diff_thr);
    }
    if (root.contains("downsample")) {
        cfg.downsample.voxel_size_m =
            pick(root.at("downsample"), "voxel_size_m", cfg.downsample.voxel_size_m);
    }
    cfg.start_idx    = pick(root, "start_idx",    cfg.start_idx);
    cfg.end_idx      = pick(root, "end_idx",      cfg.end_idx);
    cfg.keyframe_gap = pick(root, "keyframe_gap", cfg.keyframe_gap);
    cfg.use_gpu      = pick(root, "use_gpu",      cfg.use_gpu);
    cfg.output_dir   = pick(root, "output_dir",   cfg.output_dir);
    cfg.write_pcd    = pick(root, "write_pcd",    cfg.write_pcd);
    cfg.write_hdf5   = pick(root, "write_hdf5",   cfg.write_hdf5);
    return cfg;
}

json visibility_to_json(VisibilityConfig const& v) {
    return json{
        {"top_k", v.top_k},
        {"log_odds_prior", v.log_odds_prior},
        {"log_odds_carve", v.log_odds_carve},
        {"log_odds_hit",   v.log_odds_hit},
        {"decision_thr",   v.decision_thr},
        {"log_odds_min",   v.log_odds_min},
        {"log_odds_max",   v.log_odds_max},
    };
}

}  // namespace

PipelineConfig load_config_from_json_file(std::filesystem::path const& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open config file: " + path.string());
    json j; f >> j;
    return from_json(j);
}

PipelineConfig load_config_from_json_string(std::string const& s) {
    return from_json(json::parse(s));
}

std::string dump_config_to_json(PipelineConfig const& cfg) {
    json out;
    out["sensor"] = {
        {"vfov_deg", cfg.sensor.vfov_deg},
        {"hfov_deg", cfg.sensor.hfov_deg},
    };
    out["remove"] = {
        {"resolutions_deg", cfg.remove.resolutions_deg},
        {"diff_carve_m",    cfg.remove.diff_carve_m},
        {"diff_hit_m",      cfg.remove.diff_hit_m},
        {"visibility",      visibility_to_json(cfg.remove.visibility)},
    };
    out["revert"] = {
        {"resolutions_deg", cfg.revert.resolutions_deg},
        {"diff_carve_m",    cfg.revert.diff_carve_m},
        {"diff_hit_m",      cfg.revert.diff_hit_m},
        {"visibility",      visibility_to_json(cfg.revert.visibility)},
    };
    out["knn"] = {
        {"k",                      cfg.knn.k},
        {"radius_m",               cfg.knn.radius_m},
        {"scan_map_avg_diff_thr",  cfg.knn.scan_map_avg_diff_thr},
    };
    out["downsample"]   = {{"voxel_size_m", cfg.downsample.voxel_size_m}};
    out["start_idx"]    = cfg.start_idx;
    out["end_idx"]      = cfg.end_idx;
    out["keyframe_gap"] = cfg.keyframe_gap;
    out["use_gpu"]      = cfg.use_gpu;
    out["output_dir"]   = cfg.output_dir;
    out["write_pcd"]    = cfg.write_pcd;
    out["write_hdf5"]   = cfg.write_hdf5;
    return out.dump(2);
}

}  // namespace removert::io
