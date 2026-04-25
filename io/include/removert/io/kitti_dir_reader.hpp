// KITTI velodyne directory reader (.bin scans + poses.txt + calib.txt).
//
// Layout expected:
//   <root>/velodyne/000000.bin   (Float4 x N, x,y,z,intensity)
//   <root>/poses.txt             (3x4 floats per line, cam0 frame)
//   <root>/calib.txt             (KITTI odometry style; supplies Tr velo->cam0)
//
// `pose_world_lidar` for frame i is constructed as:
//   poses[i] @ Tr   (pose semantics matches SuMa / SemanticKITTI).

#pragma once

#include "removert/io/dataset_reader.hpp"

#include <Eigen/Core>
#include <filesystem>
#include <string>
#include <vector>

namespace removert::io {

class KittiDirReader final : public IDatasetReader {
public:
    explicit KittiDirReader(std::filesystem::path const& root,
                            SensorConfig sensor = default_kitti_sensor());

    [[nodiscard]] std::size_t  size()   const override { return scan_paths_.size(); }
    [[nodiscard]] SensorConfig sensor() const override { return sensor_; }
    [[nodiscard]] ScanFrame    read(std::size_t idx) const override;

    // Reasonable defaults for a 64-line Velodyne (e.g. HDL-64E) on KITTI.
    [[nodiscard]] static SensorConfig default_kitti_sensor();

private:
    SensorConfig                          sensor_;
    Eigen::Matrix4f                       Tr_velo_cam_{Eigen::Matrix4f::Identity()};
    std::vector<Eigen::Matrix4f>          poses_cam0_;
    std::vector<std::filesystem::path>    scan_paths_;
};

}  // namespace removert::io
