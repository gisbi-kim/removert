// HDF5 output writer (plan §3.6, §P7).
//
//   /static_map    (Ns, 4) float32
//   /dynamic_map   (Nd, 4) float32
//   /log_odds      (Nm,)   float32
//
// Files are gzip-compressed (level 4) chunked.

#pragma once

#if REMOVERT_WITH_HDF5

#include "removert/types.hpp"

#include <filesystem>
#include <vector>

namespace removert::io {

void write_pipeline_output_h5(std::filesystem::path const& path,
                              std::vector<Point> const& static_map,
                              std::vector<Point> const& dynamic_map,
                              std::vector<float> const& log_odds);

}  // namespace removert::io

#endif  // REMOVERT_WITH_HDF5
