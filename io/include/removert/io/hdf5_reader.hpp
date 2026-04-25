// HDF5 dataset reader. Schema (see plan §3.5):
//
//   /sensor                 group with attrs vfov_deg, hfov_deg, T_pose_lidar
//   /poses                  (M, 16) float32 row-major
//   /timestamps_ns          (M,)    int64
//   /scans/{i:06d}/points   (Ni, 4) float32 (x, y, z, intensity)
//
// Implemented against the HDF5 C++ API (H5Cpp.h).

#pragma once

#if REMOVERT_WITH_HDF5

#include "removert/io/dataset_reader.hpp"

#include <Eigen/Core>
#include <filesystem>
#include <memory>
#include <vector>

namespace H5 { class H5File; }

namespace removert::io {

class Hdf5DatasetReader final : public IDatasetReader {
public:
    explicit Hdf5DatasetReader(std::filesystem::path const& path);
    ~Hdf5DatasetReader() override;

    Hdf5DatasetReader(Hdf5DatasetReader const&)            = delete;
    Hdf5DatasetReader& operator=(Hdf5DatasetReader const&) = delete;
    Hdf5DatasetReader(Hdf5DatasetReader&&) noexcept;
    Hdf5DatasetReader& operator=(Hdf5DatasetReader&&) noexcept;

    [[nodiscard]] std::size_t  size()   const override { return frame_count_; }
    [[nodiscard]] SensorConfig sensor() const override { return sensor_; }
    [[nodiscard]] ScanFrame    read(std::size_t idx) const override;

private:
    std::unique_ptr<H5::H5File>     file_;
    SensorConfig                    sensor_{};
    std::size_t                     frame_count_{0};
    std::vector<Eigen::Matrix4f>    poses_;
    std::vector<std::int64_t>       stamps_;
};

}  // namespace removert::io

#endif  // REMOVERT_WITH_HDF5
