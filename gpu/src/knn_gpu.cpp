// cuVS-backed brute-force KNN (plan §4.4 / §4.7).
//
// Two implementations live behind one entry point:
//   * cuvs_brute_force — preferred path. Activated when the build links
//     against cuvs 24.10+ (CMake sets REMOVERT_HAVE_CUVS=1).
//   * cpu_brute_force  — partial_sort over squared distances. Always
//     available so unit tests still run on CPU-only environments.
//
// TODO(local-test): UNTESTED ON GPU HARDWARE. The cuVS API surface has
// shifted across 24.04 / 24.06 / 24.08 / 24.10 releases; if the build is
// linking an older cuvs, the function names / namespaces below may need a
// nudge. See https://docs.rapids.ai/api/cuvs/stable/cpp_api/neighbors_bruteforce/

#include "removert/gpu/knn_gpu.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <vector>

#if REMOVERT_HAVE_CUVS
#include <cuvs/distance/distance.hpp>
#include <cuvs/neighbors/brute_force.hpp>
#include <raft/core/device_mdspan.hpp>
#include <raft/core/device_resources.hpp>
#include <rmm/device_uvector.hpp>
#include <cuda_runtime.h>
#endif

namespace removert::gpu {

namespace {

// -- CPU fallback -----------------------------------------------------------

float sq_dist(Point a, Point b) noexcept {
    float const dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

KnnResult cpu_brute_force(std::vector<Point> const& dataset,
                          std::vector<Point> const& queries,
                          int k) {
    KnnResult res;
    auto const nq = queries.size();
    auto const slot = static_cast<std::size_t>(k);
    res.idx .assign(nq * slot, -1);
    res.dist.assign(nq * slot, std::numeric_limits<float>::infinity());
    if (dataset.empty() || k <= 0) return res;

    std::vector<float> ds(dataset.size());
    for (std::size_t qi = 0; qi < nq; ++qi) {
        for (std::size_t di = 0; di < dataset.size(); ++di)
            ds[di] = sq_dist(queries[qi], dataset[di]);

        std::vector<int> order(dataset.size());
        std::iota(order.begin(), order.end(), 0);
        int const kk = std::min<int>(k, static_cast<int>(dataset.size()));
        std::partial_sort(
            order.begin(), order.begin() + kk, order.end(),
            [&](int a, int b) {
                return ds[static_cast<std::size_t>(a)]
                     < ds[static_cast<std::size_t>(b)];
            });
        for (int j = 0; j < kk; ++j) {
            std::size_t const out = qi * slot + static_cast<std::size_t>(j);
            int const  idx = order[static_cast<std::size_t>(j)];
            res.idx [out] = idx;
            res.dist[out] = std::sqrt(ds[static_cast<std::size_t>(idx)]);
        }
    }
    return res;
}

// -- GPU path (cuVS) --------------------------------------------------------

#if REMOVERT_HAVE_CUVS

// Pack Points (x, y, z, intensity) → contiguous (N, 3) float for cuVS. We drop
// intensity so it does not pollute the L2 distance.
std::vector<float> pack_xyz(std::vector<Point> const& pts) {
    std::vector<float> out(pts.size() * 3);
    for (std::size_t i = 0; i < pts.size(); ++i) {
        out[3 * i + 0] = pts[i].x;
        out[3 * i + 1] = pts[i].y;
        out[3 * i + 2] = pts[i].z;
    }
    return out;
}

KnnResult cuvs_brute_force(std::vector<Point> const& dataset,
                           std::vector<Point> const& queries,
                           int k) {
    constexpr int dim = 3;
    raft::device_resources handle;
    auto stream = handle.get_stream();

    auto const h_d = pack_xyz(dataset);
    auto const h_q = pack_xyz(queries);

    rmm::device_uvector<float> d_dataset(h_d.size(), stream);
    rmm::device_uvector<float> d_queries(h_q.size(), stream);
    cudaMemcpyAsync(d_dataset.data(), h_d.data(),
                    sizeof(float) * h_d.size(), cudaMemcpyHostToDevice, stream);
    cudaMemcpyAsync(d_queries.data(), h_q.data(),
                    sizeof(float) * h_q.size(), cudaMemcpyHostToDevice, stream);

    auto dataset_view = raft::make_device_matrix_view<const float, int64_t>(
        d_dataset.data(), static_cast<int64_t>(dataset.size()), dim);
    auto queries_view = raft::make_device_matrix_view<const float, int64_t>(
        d_queries.data(), static_cast<int64_t>(queries.size()), dim);

    auto index = cuvs::neighbors::brute_force::build(
        handle, dataset_view, cuvs::distance::DistanceType::L2Expanded);

    auto const nq = queries.size();
    rmm::device_uvector<int64_t> d_idx (nq * k, stream);
    rmm::device_uvector<float>   d_dist(nq * k, stream);
    auto idx_view = raft::make_device_matrix_view<int64_t, int64_t>(
        d_idx.data(), static_cast<int64_t>(nq), k);
    auto dist_view = raft::make_device_matrix_view<float, int64_t>(
        d_dist.data(), static_cast<int64_t>(nq), k);

    cuvs::neighbors::brute_force::search(handle, index,
                                         queries_view, idx_view, dist_view);

    std::vector<int64_t> h_idx (nq * k);
    std::vector<float>   h_dist(nq * k);
    cudaMemcpyAsync(h_idx .data(), d_idx .data(),
                    sizeof(int64_t) * h_idx.size(),
                    cudaMemcpyDeviceToHost, stream);
    cudaMemcpyAsync(h_dist.data(), d_dist.data(),
                    sizeof(float) * h_dist.size(),
                    cudaMemcpyDeviceToHost, stream);
    handle.sync_stream();

    KnnResult out;
    out.idx .resize(h_idx.size());
    out.dist.resize(h_dist.size());
    for (std::size_t i = 0; i < h_idx.size(); ++i) {
        out.idx [i] = static_cast<int>(h_idx[i]);
        // L2Expanded returns squared distances; CPU path returns Euclidean,
        // so take sqrt to keep both backends behaviourally equivalent.
        out.dist[i] = std::sqrt(h_dist[i]);
    }
    return out;
}

#endif  // REMOVERT_HAVE_CUVS

}  // namespace

KnnResult brute_force_knn(std::vector<Point> const& dataset,
                          std::vector<Point> const& queries,
                          int k) {
#if REMOVERT_HAVE_CUVS
    if (!dataset.empty() && !queries.empty() && k > 0) {
        return cuvs_brute_force(dataset, queries, k);
    }
#endif
    return cpu_brute_force(dataset, queries, k);
}

}  // namespace removert::gpu
