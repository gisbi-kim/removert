// cuVS-backed brute-force KNN (plan §4.4 / §4.7).
//
// When linked against cuvs 24.10+ the implementation calls
// cuvs::neighbors::brute_force::{build, search} on a raft::device_resources
// stream. Otherwise a CPU partial_sort fallback is used so the function is
// always callable. TODO(local-test): cuVS path not yet GPU-tested.

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
