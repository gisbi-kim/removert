// SKELETON for the fused projection + carving kernel. Real implementation in P5.
//
// What this lands today:
//   * a buildable .cu translation unit so the GPU target links,
//   * the public API surface accepting device pointers,
//   * a TODO body that explicitly fails at runtime (no silent regressions).

#include "removert/gpu/projection.cuh"

#include <cuda_runtime.h>

namespace removert::gpu {

void run_fused_projection(Point const* /*d_map*/, int /*n_map*/,
                          Point const* /*d_scan*/, int /*n_scan*/,
                          float const* /*d_T*/,
                          int /*rows*/, int /*cols*/, int /*top_k*/,
                          float /*res*/,
                          float /*vfov*/, float /*hfov*/,
                          float /*carve_m*/, float /*hit_m*/,
                          float /*l_carve*/, float /*l_hit*/,
                          float /*lo_min*/, float /*lo_max*/,
                          float* /*state*/) {
    // TODO(P5): launch fused projection + carving kernel here.
    // Until the kernel lands, callers must select the CPU backend.
}

}  // namespace removert::gpu
