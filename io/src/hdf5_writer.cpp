#include "removert/io/hdf5_writer.hpp"

#if REMOVERT_WITH_HDF5

#include <H5Cpp.h>

#include <algorithm>
#include <stdexcept>

namespace removert::io {

namespace {

void write_points(H5::H5File& f, std::string const& name,
                  std::vector<Point> const& pts) {
    hsize_t const N = pts.size();
    hsize_t dims[2] = {N, 4};
    H5::DataSpace space(2, dims);

    H5::DSetCreatPropList plist;
    if (N > 0) {
        hsize_t const chunk_n = std::min<hsize_t>(N, 65536);
        hsize_t chunk[2] = {chunk_n, 4};
        plist.setChunk(2, chunk);
        plist.setDeflate(4);
    }
    H5::DataSet ds = f.createDataSet(name, H5::PredType::NATIVE_FLOAT,
                                     space, plist);
    if (N > 0) ds.write(pts.data(), H5::PredType::NATIVE_FLOAT);
}

void write_floats(H5::H5File& f, std::string const& name,
                  std::vector<float> const& v) {
    hsize_t const N = v.size();
    hsize_t dims[1] = {N};
    H5::DataSpace space(1, dims);

    H5::DSetCreatPropList plist;
    if (N > 0) {
        hsize_t const chunk_n = std::min<hsize_t>(N, 1u << 20);
        hsize_t chunk[1] = {chunk_n};
        plist.setChunk(1, chunk);
        plist.setDeflate(4);
    }
    H5::DataSet ds = f.createDataSet(name, H5::PredType::NATIVE_FLOAT,
                                     space, plist);
    if (N > 0) ds.write(v.data(), H5::PredType::NATIVE_FLOAT);
}

}  // namespace

void write_pipeline_output_h5(std::filesystem::path const& path,
                              std::vector<Point> const& static_map,
                              std::vector<Point> const& dynamic_map,
                              std::vector<float> const& log_odds) {
    H5::Exception::dontPrint();
    H5::H5File f(path.string(), H5F_ACC_TRUNC);
    write_points(f, "/static_map",  static_map);
    write_points(f, "/dynamic_map", dynamic_map);
    write_floats(f, "/log_odds",    log_odds);
}

}  // namespace removert::io

#endif  // REMOVERT_WITH_HDF5
