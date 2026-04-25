#include "removert/io/pcd_writer.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

TEST(PcdWriter, WritesHeaderAndPoints) {
    fs::path tmp = fs::temp_directory_path() / "removert_test.pcd";
    std::vector<removert::Point> pts{
        {1.0f, 2.0f, 3.0f, 0.5f},
        {4.0f, 5.0f, 6.0f, 0.25f}};
    ASSERT_TRUE(removert::io::write_pcd_binary(tmp, pts));

    std::ifstream f(tmp, std::ios::binary);
    std::stringstream ss; ss << f.rdbuf();
    std::string buf = ss.str();
    EXPECT_NE(buf.find("VERSION 0.7"), std::string::npos);
    EXPECT_NE(buf.find("POINTS 2"),     std::string::npos);
    EXPECT_NE(buf.find("DATA binary"),  std::string::npos);
    auto data_pos = buf.find("DATA binary\n");
    ASSERT_NE(data_pos, std::string::npos);
    auto payload_off = data_pos + std::string("DATA binary\n").size();
    EXPECT_GE(buf.size() - payload_off, 2u * sizeof(removert::Point));
    fs::remove(tmp);
}
