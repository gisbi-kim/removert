// Fused projection + carving + log-odds update CUDA kernel (plan §4.2/§4.8).
//
// Pipeline (one call to run_fused_projection):
//   1. fill_inf_kernel       — clear scratch scan range image to +inf
//   2. build_scan_rimg       — 1 thread / scan point: project + atomic-min onto
//                              scan range image (libcudacxx atomic_ref)
//   3. visibility_update     — 1 thread / map point: transform world→sensor,
//                              project, look up scan_rimg at the pixel,
//                              classify (hit | carve | nothing) and accumulate
//                              into per-map-point log_odds with clamping
//
// All three steps run on the same stream and synchronize at the end.
//
// TODO(local-test): UNTESTED ON GPU HARDWARE. Compile-checked only by
// inspection. First real build will almost certainly surface
// nvcc/libcudacxx/atomic_ref version drift. See PR #34 for validation guide.

#include "removert/gpu/projection.cuh"

#include <cuda/atomic>
#include <cuda_runtime.h>

#include <cmath>
#include <cstdio>

namespace removert::gpu {

namespace {

constexpr float kPi      = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;
constexpr float kRangeEps = 1.0e-6f;

#define REMOVERT_CUDA_CHECK(call)                                              \
    do {                                                                       \
        cudaError_t _e = (call);                                               \
        if (_e != cudaSuccess) {                                               \
            std::fprintf(stderr, "CUDA %s @%s:%d: %s\n",                       \
                         #call, __FILE__, __LINE__, cudaGetErrorString(_e));   \
            return;                                                            \
        }                                                                      \
    } while (0)

// Atomic min on a float location.  Range values are non-negative, so the
// usual signed-zero / NaN edge-cases of bit-cast min do not apply here.
__device__ inline void atomic_min_relaxed(float* addr, float val) {
    cuda::atomic_ref<float, cuda::thread_scope_device> ref(*addr);
    float cur = ref.load(cuda::memory_order_relaxed);
    while (val < cur) {
        if (ref.compare_exchange_weak(cur, val,
                                       cuda::memory_order_relaxed,
                                       cuda::memory_order_relaxed)) return;
    }
}

// Cartesian → range-image pixel.  Returns false if outside the configured FOV
// or the projected pixel falls outside the integer grid.
__device__ inline bool to_pixel(float x, float y, float z,
                                int rows, int cols,
                                float vfov_rad, float hfov_rad,
                                int& i_out, int& j_out, float& r_out) {
    float r2 = x * x + y * y + z * z;
    if (r2 < kRangeEps) return false;
    float r  = sqrtf(r2);
    float az = atan2f(y, x);
    float el = asinf(z / r);
    if (fabsf(az) > hfov_rad * 0.5f) return false;
    if (fabsf(el) > vfov_rad * 0.5f) return false;
    int i = __float2int_rn((1.0f - (el + vfov_rad * 0.5f) / vfov_rad)
                           * static_cast<float>(rows - 1));
    int j = __float2int_rn((az / hfov_rad + 0.5f)
                           * static_cast<float>(cols - 1));
    if (i < 0 || i >= rows || j < 0 || j >= cols) return false;
    i_out = i;
    j_out = j;
    r_out = r;
    return true;
}

__global__ void fill_inf_kernel(float* p, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) p[idx] = INFINITY;
}

__global__ void build_scan_rimg_kernel(Point const* __restrict__ scan, int n,
                                       float* __restrict__ rimg,
                                       int rows, int cols,
                                       float vfov_rad, float hfov_rad) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n) return;
    Point p = scan[idx];
    int i, j;
    float r;
    if (!to_pixel(p.x, p.y, p.z, rows, cols, vfov_rad, hfov_rad, i, j, r))
        return;
    atomic_min_relaxed(&rimg[i * cols + j], r);
}

__global__ void visibility_update_kernel(
        Point const* __restrict__ map, int n_map,
        float const* __restrict__ T,           // 16 floats, row-major, world→sensor
        float const* __restrict__ scan_rimg,
        int rows, int cols,
        float vfov_rad, float hfov_rad,
        float carve_m, float hit_m,
        float l_carve, float l_hit,
        float lo_min, float lo_max,
        float* __restrict__ log_odds) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n_map) return;
    Point pw = map[idx];
    // Apply the upper-3x4 of T (row-major) to bring the world-frame map point
    // into the current scan's sensor frame.
    float xs = T[0]*pw.x + T[1]*pw.y + T[2 ]*pw.z + T[3 ];
    float ys = T[4]*pw.x + T[5]*pw.y + T[6 ]*pw.z + T[7 ];
    float zs = T[8]*pw.x + T[9]*pw.y + T[10]*pw.z + T[11];

    int i, j;
    float r_map;
    if (!to_pixel(xs, ys, zs, rows, cols, vfov_rad, hfov_rad, i, j, r_map))
        return;

    float r_scan = scan_rimg[i * cols + j];
    if (!isfinite(r_scan)) return;            // no scan return at this pixel

    float diff = r_map - r_scan;              // >0  → map point is behind scan
    float delta = 0.0f;
    if (fabsf(diff) <= hit_m) {
        // Map point matches the scan return: positive (static) evidence (A3).
        delta = l_hit;
    } else if (diff < -carve_m) {
        // Map point sits in the free segment that the ray traversed (A2):
        // negative (dynamic) evidence.
        delta = l_carve;
    }
    // diff > hit_m: map point is occluded by something the scan saw; we have
    // no visibility information about it this frame → no update.
    if (delta == 0.0f) return;

    // Clamped atomic add into per-map-point log-odds buffer.
    cuda::atomic_ref<float, cuda::thread_scope_device> ref(log_odds[idx]);
    float cur = ref.load(cuda::memory_order_relaxed);
    while (true) {
        float next = fmaxf(lo_min, fminf(lo_max, cur + delta));
        if (ref.compare_exchange_weak(cur, next,
                                       cuda::memory_order_relaxed,
                                       cuda::memory_order_relaxed)) return;
    }
}

}  // namespace

void run_fused_projection(Point const* d_map_pts,  int n_map,
                          Point const* d_scan_pts, int n_scan,
                          float const* d_T_lidar_world,
                          int rows, int cols, int /*top_k*/,
                          float /*resolution_deg*/,
                          float vfov_deg, float hfov_deg,
                          float diff_carve_m, float diff_hit_m,
                          float carve_log_odds, float hit_log_odds,
                          float lo_min, float lo_max,
                          float* state_log_odds) {
    if (n_map <= 0 || rows <= 0 || cols <= 0 || state_log_odds == nullptr)
        return;

    int const npix  = rows * cols;
    int const block = 256;
    float const vfov_rad = vfov_deg * kDeg2Rad;
    float const hfov_rad = hfov_deg * kDeg2Rad;

    // Scratch scan range image lives only for the duration of this call.
    float* d_scan_rimg = nullptr;
    REMOVERT_CUDA_CHECK(cudaMalloc(&d_scan_rimg, sizeof(float) * npix));

    fill_inf_kernel<<<(npix + block - 1) / block, block>>>(d_scan_rimg, npix);

    if (n_scan > 0 && d_scan_pts != nullptr) {
        build_scan_rimg_kernel<<<(n_scan + block - 1) / block, block>>>(
            d_scan_pts, n_scan, d_scan_rimg, rows, cols, vfov_rad, hfov_rad);
    }

    visibility_update_kernel<<<(n_map + block - 1) / block, block>>>(
        d_map_pts, n_map, d_T_lidar_world, d_scan_rimg, rows, cols,
        vfov_rad, hfov_rad,
        diff_carve_m, diff_hit_m,
        carve_log_odds, hit_log_odds,
        lo_min, lo_max,
        state_log_odds);

    REMOVERT_CUDA_CHECK(cudaDeviceSynchronize());
    REMOVERT_CUDA_CHECK(cudaFree(d_scan_rimg));
}

}  // namespace removert::gpu
