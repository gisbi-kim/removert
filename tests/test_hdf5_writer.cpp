// Smoke test for the HDF5 writer. Only built when REMOVERT_WITH_HDF5 is on.
//
// We write a tiny output, reopen via the C++ API, and check sizes/values.

#include "removert/io/hdf5_writer.hpp"

#include <gtest/gtest.h>

#if REMOVERT_WITH_HDF5

#include <H5Cpp.h>

#include <filesystem>

namespace fs = std::filesystem;

TEST(Hdf5Writer, WritesStaticDynamicLogOdds) {
    fs::path tmp = fs::temp_directory_path() / "removert_writer_test.h5";
    std::vector<removert::Point> stat{
        {1.0f, 2.0f, 3.0f, 0.0f},
        {4.0f, 5.0f, 6.0f, 0.0f}};
    std::vector<removert::Point> dyn{{7.0f, 8.0f, 9.0f, 1.0f}};
    std::vector<float> lo{0.0f, 1.0f, -1.0f, 2.0f};

    removert::io::write_pipeline_output_h5(tmp, stat, dyn, lo);
    ASSERT_TRUE(fs::exists(tmp));

    H5::Exception::dontPrint();
    H5::H5File f(tmp.string(), H5F_ACC_RDONLY);
    {
        auto ds = f.openDataSet("/static_map");
        hsize_t dims[2]{0, 0};
        ds.getSpace().getSimpleExtentDims(dims);
        EXPECT_EQ(dims[0], 2u);
        EXPECT_EQ(dims[1], 4u);
    }
    {
        auto ds = f.openDataSet("/dynamic_map");
        hsize_t dims[2]{0, 0};
        ds.getSpace().getSimpleExtentDims(dims);
        EXPECT_EQ(dims[0], 1u);
    }
    {
        auto ds = f.openDataSet("/log_odds");
        hsize_t dims[1]{0};
        ds.getSpace().getSimpleExtentDims(dims);
        EXPECT_EQ(dims[0], 4u);
    }
    fs::remove(tmp);
}

#endif  // REMOVERT_WITH_HDF5
