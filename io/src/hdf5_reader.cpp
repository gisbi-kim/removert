#include "removert/io/hdf5_reader.hpp"

#if REMOVERT_WITH_HDF5

#include <H5Cpp.h>

#include <cstdio>
#include <stdexcept>

namespace removert::io {

namespace {

std::string scan_group_name(std::size_t i) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "/scans/%06zu/points", i);
    return buf;
}

void read_sensor_attrs(H5::H5File& f, SensorConfig& s) {
    if (!f.exists("/sensor")) return;
    H5::Group g = f.openGroup("/sensor");
    if (g.attrExists("vfov_deg"))
        g.openAttribute("vfov_deg").read(H5::PredType::NATIVE_FLOAT, &s.vfov_deg);
    if (g.attrExists("hfov_deg"))
        g.openAttribute("hfov_deg").read(H5::PredType::NATIVE_FLOAT, &s.hfov_deg);
    if (g.attrExists("T_pose_lidar")) {
        float buf[16];
        g.openAttribute("T_pose_lidar").read(H5::PredType::NATIVE_FLOAT, buf);
        Eigen::Matrix<float, 4, 4, Eigen::RowMajor> rm;
        std::memcpy(rm.data(), buf, sizeof(buf));
        s.T_pose_lidar = rm;
    }
}

void read_poses(H5::H5File& f, std::vector<Eigen::Matrix4f>& poses) {
    if (!f.exists("/poses")) return;
    H5::DataSet ds = f.openDataSet("/poses");
    H5::DataSpace sp = ds.getSpace();
    hsize_t dims[2] = {0, 0};
    sp.getSimpleExtentDims(dims);
    if (dims[1] != 16) throw std::runtime_error("/poses second dim must be 16");
    std::vector<float> buf(static_cast<std::size_t>(dims[0]) * 16);
    ds.read(buf.data(), H5::PredType::NATIVE_FLOAT);
    poses.resize(static_cast<std::size_t>(dims[0]));
    for (std::size_t i = 0; i < poses.size(); ++i) {
        Eigen::Matrix<float, 4, 4, Eigen::RowMajor> rm;
        std::memcpy(rm.data(), buf.data() + i * 16, 16 * sizeof(float));
        poses[i] = rm;
    }
}

void read_stamps(H5::H5File& f, std::vector<std::int64_t>& stamps) {
    if (!f.exists("/timestamps_ns")) return;
    H5::DataSet ds = f.openDataSet("/timestamps_ns");
    hsize_t dims[1] = {0};
    ds.getSpace().getSimpleExtentDims(dims);
    stamps.resize(static_cast<std::size_t>(dims[0]));
    ds.read(stamps.data(), H5::PredType::NATIVE_INT64);
}

}  // namespace

Hdf5DatasetReader::Hdf5DatasetReader(std::filesystem::path const& path) {
    H5::Exception::dontPrint();
    file_ = std::make_unique<H5::H5File>(path.string(), H5F_ACC_RDONLY);
    read_sensor_attrs(*file_, sensor_);
    read_poses (*file_, poses_);
    read_stamps(*file_, stamps_);
    frame_count_ = poses_.size();
    if (sensor_.vfov_deg <= 0.0f) sensor_.vfov_deg = 50.0f;
    if (sensor_.hfov_deg <= 0.0f) sensor_.hfov_deg = 360.0f;
}

Hdf5DatasetReader::~Hdf5DatasetReader() = default;
Hdf5DatasetReader::Hdf5DatasetReader(Hdf5DatasetReader&&) noexcept            = default;
Hdf5DatasetReader& Hdf5DatasetReader::operator=(Hdf5DatasetReader&&) noexcept = default;

ScanFrame Hdf5DatasetReader::read(std::size_t idx) const {
    if (idx >= frame_count_) throw std::out_of_range("Hdf5DatasetReader::read");
    ScanFrame f;
    f.T_world_lidar.T = poses_[idx];
    f.stamp_ns = (idx < stamps_.size()) ? stamps_[idx] : 0;

    H5::DataSet ds = file_->openDataSet(scan_group_name(idx));
    hsize_t dims[2] = {0, 0};
    ds.getSpace().getSimpleExtentDims(dims);
    if (dims[1] != 4) throw std::runtime_error("scan dataset second dim must be 4");
    f.points_local.resize(static_cast<std::size_t>(dims[0]));
    if (!f.points_local.empty())
        ds.read(f.points_local.data(), H5::PredType::NATIVE_FLOAT);
    return f;
}

}  // namespace removert::io

#endif  // REMOVERT_WITH_HDF5
