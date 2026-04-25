#include "removert/io/pcd_writer.hpp"

#include <cstdio>

namespace removert::io {

bool write_pcd_binary(std::filesystem::path const& path,
                      std::vector<Point> const& pts) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;

    std::fprintf(f,
        "# .PCD v0.7 - removert v2 binary PCD\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH %zu\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %zu\n"
        "DATA binary\n",
        pts.size(), pts.size());

    if (!pts.empty())
        std::fwrite(pts.data(), sizeof(Point), pts.size(), f);
    std::fclose(f);
    return true;
}

}  // namespace removert::io
