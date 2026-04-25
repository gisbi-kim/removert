#include "removert/io/kitti_dir_reader.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace removert::io {

namespace {

namespace fs = std::filesystem;

Eigen::Matrix4f read_3x4_to_4x4(std::istringstream& iss) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            float v;
            if (!(iss >> v)) throw std::runtime_error("KITTI parse: short row");
            T(r, c) = v;
        }
    }
    return T;
}

std::vector<Eigen::Matrix4f> parse_poses(fs::path const& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open poses.txt: " + path.string());
    std::vector<Eigen::Matrix4f> out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        out.push_back(read_3x4_to_4x4(iss));
    }
    return out;
}

Eigen::Matrix4f parse_tr(fs::path const& calib_path) {
    std::ifstream f(calib_path);
    if (!f) return Eigen::Matrix4f::Identity();
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("Tr:", 0) != 0 && line.rfind("Tr ", 0) != 0) continue;
        auto colon = line.find(':');
        std::istringstream iss(line.substr(colon + 1));
        return read_3x4_to_4x4(iss);
    }
    return Eigen::Matrix4f::Identity();
}

std::vector<fs::path> list_velodyne_bins(fs::path const& dir) {
    std::vector<fs::path> out;
    if (!fs::exists(dir)) return out;
    for (auto const& e : fs::directory_iterator(dir))
        if (e.path().extension() == ".bin") out.push_back(e.path());
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<Point> read_bin_scan(fs::path const& p) {
    std::vector<Point> pts;
    std::FILE* f = std::fopen(p.c_str(), "rb");
    if (!f) throw std::runtime_error("cannot open scan: " + p.string());
    std::fseek(f, 0, SEEK_END);
    auto const bytes = static_cast<std::size_t>(std::ftell(f));
    std::fseek(f, 0, SEEK_SET);
    pts.resize(bytes / sizeof(Point));
    if (!pts.empty()) {
        auto const got = std::fread(pts.data(), sizeof(Point), pts.size(), f);
        (void)got;
    }
    std::fclose(f);
    return pts;
}

}  // namespace

SensorConfig KittiDirReader::default_kitti_sensor() {
    SensorConfig s;
    s.vfov_deg = 50.0f;       // HDL-64E ~+2 / -24.8 -> ~27 deg actual,
                              //    we widen to be safe; tweak via Config.
    s.hfov_deg = 360.0f;
    return s;
}

KittiDirReader::KittiDirReader(fs::path const& root, SensorConfig sensor)
    : sensor_(sensor) {
    Tr_velo_cam_  = parse_tr(root / "calib.txt");
    poses_cam0_   = parse_poses(root / "poses.txt");
    scan_paths_   = list_velodyne_bins(root / "velodyne");
    if (scan_paths_.empty())
        throw std::runtime_error("no velodyne/.bin found under " + root.string());
}

ScanFrame KittiDirReader::read(std::size_t idx) const {
    if (idx >= scan_paths_.size())
        throw std::out_of_range("KittiDirReader::read");
    ScanFrame f;
    f.points_local = read_bin_scan(scan_paths_[idx]);
    Eigen::Matrix4f const T_pose =
        (idx < poses_cam0_.size()) ? poses_cam0_[idx] : Eigen::Matrix4f::Identity();
    // pose @ Tr = world(=cam0 frame 0) <- velodyne
    f.T_world_lidar.T = T_pose * Tr_velo_cam_;
    f.stamp_ns = 0;
    return f;
}

}  // namespace removert::io
