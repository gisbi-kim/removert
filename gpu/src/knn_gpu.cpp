// SKELETON KNN backend.  P6 lands the cuVS brute-force build + search.

#include "removert/gpu/knn_gpu.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace removert::gpu {

namespace {

float sq_dist(Point a, Point b) noexcept {
    float const dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

}  // namespace

KnnResult brute_force_knn(std::vector<Point> const& dataset,
                          std::vector<Point> const& queries,
                          int k) {
#if REMOVERT_HAVE_CUVS
    // TODO(P6): cuVS brute_force build + search.
    // For now fall through to the CPU path so output is meaningful even when
    // cuVS is linked but the wiring is incomplete.
#endif
    KnnResult res;
    res.idx .assign(queries.size() * static_cast<std::size_t>(k), -1);
    res.dist.assign(queries.size() * static_cast<std::size_t>(k),
                    std::numeric_limits<float>::infinity());
    if (dataset.empty() || k <= 0) return res;

    std::vector<float> ds(dataset.size());
    for (std::size_t qi = 0; qi < queries.size(); ++qi) {
        for (std::size_t di = 0; di < dataset.size(); ++di)
            ds[di] = sq_dist(queries[qi], dataset[di]);

        std::vector<int> order(dataset.size());
        std::iota(order.begin(), order.end(), 0);
        int const kk = std::min<int>(k, static_cast<int>(dataset.size()));
        std::partial_sort(order.begin(), order.begin() + kk, order.end(),
                          [&](int a, int b) { return ds[static_cast<std::size_t>(a)]
                                                   < ds[static_cast<std::size_t>(b)]; });
        for (int j = 0; j < kk; ++j) {
            res.idx [qi * static_cast<std::size_t>(k) + static_cast<std::size_t>(j)] = order[static_cast<std::size_t>(j)];
            res.dist[qi * static_cast<std::size_t>(k) + static_cast<std::size_t>(j)] =
                std::sqrt(ds[static_cast<std::size_t>(order[static_cast<std::size_t>(j)])]);
        }
    }
    return res;
}

}  // namespace removert::gpu
