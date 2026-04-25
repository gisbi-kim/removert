// SQLite dataset reader. Schema follows tools/build_seq01_sqlite.py exactly:
//
//   meta(key, value)
//   calib(key, rows, cols, data BLOB)         (float64)
//   frames(frame_idx, timestamp_ns, pose_cam0 BLOB)   pose = 4x4 float32 row-major
//   scans (frame_idx, num_points, points BLOB)        (N, 4) float32 x,y,z,intensity
//   images(frame_idx, cam_id, w, h, format, data BLOB)
//
// Pose semantics (per build_seq01_sqlite.py docstring):
//   pose_cam0[i] : cam0_frame_i  ->  cam0_frame_0
//   To get world<-lidar at frame i:  pose_cam0[i] @ Tr   where
//   Tr is the velodyne -> cam0 transform (calib keys velo_to_cam_R / _T).

#pragma once

#if REMOVERT_WITH_SQLITE

#include "removert/io/dataset_reader.hpp"

#include <Eigen/Core>
#include <filesystem>
#include <string>
#include <vector>

struct sqlite3;

namespace removert::io {

class SqliteDatasetReader final : public IDatasetReader {
public:
    explicit SqliteDatasetReader(std::filesystem::path const& db_path,
                                 SensorConfig override_sensor = {});
    ~SqliteDatasetReader() override;

    SqliteDatasetReader(SqliteDatasetReader const&)            = delete;
    SqliteDatasetReader& operator=(SqliteDatasetReader const&) = delete;
    SqliteDatasetReader(SqliteDatasetReader&&) noexcept;
    SqliteDatasetReader& operator=(SqliteDatasetReader&&) noexcept;

    [[nodiscard]] std::size_t  size()   const override { return frame_count_; }
    [[nodiscard]] SensorConfig sensor() const override { return sensor_; }
    [[nodiscard]] ScanFrame    read(std::size_t idx) const override;

private:
    void load_calib_();
    void count_frames_();

    sqlite3*        db_{nullptr};
    SensorConfig    sensor_{};
    Eigen::Matrix4f Tr_velo_cam_{Eigen::Matrix4f::Identity()};
    std::size_t     frame_count_{0};
};

}  // namespace removert::io

#endif  // REMOVERT_WITH_SQLITE
