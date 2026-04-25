// removert_cli — single offline batch frontend for removert v2.
//
// Usage:
//   removert_cli --dataset <kitti_dir|sqlite|hdf5>
//                --input <path>
//                [--config <config.json>]
//                [--output-dir <dir>]
//
// Pipeline:
//   1. Open dataset reader.
//   2. Build initial global map (concat of every keyframe scan in world frame).
//   3. Run remove + revert pipeline.
//   4. Write static_map.pcd, dynamic_map.pcd, (optionally) output.h5.

#include "removert/geometry.hpp"
#include "removert/pipeline.hpp"
#include "removert/types.hpp"

#include "removert/io/config_loader.hpp"
#include "removert/io/dataset_reader.hpp"
#include "removert/io/kitti_dir_reader.hpp"
#include "removert/io/pcd_writer.hpp"

#if REMOVERT_WITH_SQLITE
#include "removert/io/sqlite_reader.hpp"
#endif

#if REMOVERT_WITH_HDF5
#include "removert/io/hdf5_reader.hpp"
#include "removert/io/hdf5_writer.hpp"
#endif

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace {

struct Args {
    std::string dataset_kind;     // "kitti" | "sqlite" | "hdf5"
    std::string input_path;
    std::string config_path;
    std::string output_dir;       // overrides config
    bool        help{false};
};

void print_usage() {
    std::puts(
        "removert_cli — offline batch removert v2\n"
        "\n"
        "Required:\n"
        "  --dataset <kitti|sqlite|hdf5>   choose dataset backend\n"
        "  --input <path>                  path to dataset (dir / .sqlite / .h5)\n"
        "\n"
        "Optional:\n"
        "  --config <file.json>            override defaults\n"
        "  --output-dir <dir>              where to write outputs\n"
        "  -h, --help                      show this message\n");
}

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string_view k(argv[i]);
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + std::string(k));
            return std::string(argv[++i]);
        };
        if      (k == "--dataset")    a.dataset_kind = next();
        else if (k == "--input")      a.input_path   = next();
        else if (k == "--config")     a.config_path  = next();
        else if (k == "--output-dir") a.output_dir   = next();
        else if (k == "-h" || k == "--help") a.help = true;
        else throw std::runtime_error("unknown argument: " + std::string(k));
    }
    return a;
}

std::unique_ptr<removert::io::IDatasetReader>
open_reader(std::string const& kind, std::string const& path,
            removert::SensorConfig const& sensor) {
    using namespace removert::io;
    if (kind == "kitti") {
        return std::make_unique<KittiDirReader>(path, sensor);
    }
#if REMOVERT_WITH_SQLITE
    if (kind == "sqlite") {
        return std::make_unique<SqliteDatasetReader>(path, sensor);
    }
#endif
#if REMOVERT_WITH_HDF5
    if (kind == "hdf5") {
        return std::make_unique<Hdf5DatasetReader>(path);
    }
#endif
    throw std::runtime_error("unsupported dataset kind: " + kind);
}

std::vector<removert::Point>
build_global_map(removert::io::IDatasetReader const& r,
                 removert::PipelineConfig const& cfg) {
    std::vector<removert::Point> out;
    int const begin = std::max(0, cfg.start_idx);
    int const end_  = (cfg.end_idx >= 0)
                          ? std::min<int>(cfg.end_idx, static_cast<int>(r.size()))
                          : static_cast<int>(r.size());
    int const gap = std::max(1, cfg.keyframe_gap);
    for (int i = begin; i < end_; i += gap) {
        auto frame = r.read(static_cast<std::size_t>(i));
        auto pts_w = removert::transform_all(frame.points_local,
                                             frame.T_world_lidar.T);
        out.insert(out.end(), pts_w.begin(), pts_w.end());
    }
    return out;
}

std::vector<removert::ScanFrame>
load_scans(removert::io::IDatasetReader const& r,
           removert::PipelineConfig const& cfg) {
    std::vector<removert::ScanFrame> out;
    int const end_ = (cfg.end_idx >= 0)
                         ? std::min<int>(cfg.end_idx, static_cast<int>(r.size()))
                         : static_cast<int>(r.size());
    out.reserve(static_cast<std::size_t>(end_));
    for (int i = 0; i < end_; ++i)
        out.push_back(r.read(static_cast<std::size_t>(i)));
    return out;
}

}  // namespace

int main(int argc, char** argv) try {
    auto const args = parse_args(argc, argv);
    if (args.help || args.dataset_kind.empty() || args.input_path.empty()) {
        print_usage();
        return args.help ? 0 : 2;
    }

    using namespace removert;

    PipelineConfig cfg = args.config_path.empty()
                             ? PipelineConfig{}
                             : io::load_config_from_json_file(args.config_path);
    if (!args.output_dir.empty()) cfg.output_dir = args.output_dir;

    auto reader = open_reader(args.dataset_kind, args.input_path, cfg.sensor);
    cfg.sensor = reader->sensor();
    std::cout << "[removert_cli] dataset=" << args.dataset_kind
              << " size=" << reader->size() << " scans\n";

    auto t0 = std::chrono::steady_clock::now();
    std::cout << "[removert_cli] building global map...\n";
    std::vector<Point> map_global = build_global_map(*reader, cfg);
    std::cout << "[removert_cli]   map points = " << map_global.size() << '\n';

    std::cout << "[removert_cli] loading scans...\n";
    std::vector<ScanFrame> scans = load_scans(*reader, cfg);
    std::cout << "[removert_cli]   scans loaded = " << scans.size() << '\n';

    std::cout << "[removert_cli] running remove + revert pipeline...\n";
    PipelineOutput out = remove_pipeline(std::move(map_global), scans, cfg);
    auto t1 = std::chrono::steady_clock::now();
    auto secs = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "[removert_cli]   static="  << out.static_map .size()
              <<                   " dynamic=" << out.dynamic_map.size()
              << "  (" << secs << " s)\n";

    std::filesystem::create_directories(cfg.output_dir);
    if (cfg.write_pcd) {
        io::write_pcd_binary(std::filesystem::path(cfg.output_dir) / "static_map.pcd",
                             out.static_map);
        io::write_pcd_binary(std::filesystem::path(cfg.output_dir) / "dynamic_map.pcd",
                             out.dynamic_map);
    }
#if REMOVERT_WITH_HDF5
    if (cfg.write_hdf5) {
        io::write_pipeline_output_h5(
            std::filesystem::path(cfg.output_dir) / "output.h5",
            out.static_map, out.dynamic_map, out.final_log_odds);
    }
#endif
    std::cout << "[removert_cli] done. output_dir=" << cfg.output_dir << '\n';
    return 0;
} catch (std::exception const& e) {
    std::cerr << "[removert_cli] ERROR: " << e.what() << '\n';
    return 1;
}
