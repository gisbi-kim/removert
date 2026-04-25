// Fused projection + carving + diff CUDA kernel (plan §4.2).
//
// SKELETON. Real implementation lands in P5. Interface is finalized so
// callers can be wired today.

#pragma once

#include "removert/types.hpp"

namespace removert::gpu {

// One-shot fused kernel:
//   1. transform(Points, T_lidar_world) into the sensor frame
//   2. project to range image with top-K depth peeling (cuda::atomic_ref<float>
//      min insert, libcudacxx)
//   3. carve / hit log-odds evidence directly into a per-map-point buffer
//
// `state_log_odds` lives on the GPU and is owned by the caller.
void run_fused_projection(Point const* d_map_pts, int n_map,
                          Point const* d_scan_pts, int n_scan,
                          float const* d_T_lidar_world,  // 16 floats row-major
                          int rows, int cols, int top_k,
                          float resolution_deg,
                          float vfov_deg, float hfov_deg,
                          float diff_carve_m, float diff_hit_m,
                          float carve_log_odds, float hit_log_odds,
                          float lo_min, float lo_max,
                          float* state_log_odds);

}  // namespace removert::gpu
