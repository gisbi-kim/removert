// Minimal ASCII / binary PCD writer. We deliberately avoid the PCL dependency
// for this writer — PCD's on-disk format is small and well-specified, and v1
// users only need the file to land. PCL is still optional via REMOVERT_WITH_PCL
// for users who want zero-copy PCL ingestion downstream.

#pragma once

#include "removert/types.hpp"

#include <filesystem>
#include <vector>

namespace removert::io {

// Writes binary PCD with fields x y z intensity (float32). Returns true on
// success. Always available — no PCL dependency.
bool write_pcd_binary(std::filesystem::path const& path,
                      std::vector<Point> const& pts);

}  // namespace removert::io
