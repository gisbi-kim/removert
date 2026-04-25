// cuVS-backed brute-force KNN (plan §4.4).  SKELETON for P6.

#pragma once

#include "removert/types.hpp"

#include <vector>

namespace removert::gpu {

struct KnnResult {
    std::vector<int>   idx;
    std::vector<float> dist;
};

// brute_force build + search. When REMOVERT_HAVE_CUVS=0 this is a CPU stub.
[[nodiscard]] KnnResult brute_force_knn(std::vector<Point> const& dataset,
                                        std::vector<Point> const& queries,
                                        int k);

}  // namespace removert::gpu
