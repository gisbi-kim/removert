// removert v2 — core value types.
//
// Per design charter (docs/v2-plan.md §0):
//   * No ROS, no middleware, no I/O here.
//   * Pure data — POD-ish, trivially copyable where possible.
//   * Hyperparameters live in Config aggregates only (C7).
//
// All algorithm functions operate on these types and return new values.

#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace removert {

// ---------------------------------------------------------------------------
// Geometry / sample
// ---------------------------------------------------------------------------

struct Point {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float intensity{0.0f};
};

struct SphericalPoint {
    float az{0.0f};  // radians, atan2(y, x), in (-pi, pi]
    float el{0.0f};  // radians, asin(z / r)
    float r{0.0f};   // metres
};

struct Pose {
    Eigen::Matrix4f T{Eigen::Matrix4f::Identity()};
};

// ---------------------------------------------------------------------------
// Range image
// ---------------------------------------------------------------------------
//
// A range image is a 2-D grid in (elevation, azimuth) space. We keep top-K
// nearest map points per pixel (A1: multi-hit / depth peeling, plan §4.8).
// Background pixels store +inf range and -1 source index.

inline constexpr float kNoRange = std::numeric_limits<float>::infinity();
inline constexpr int   kNoIndex = -1;

struct RangeImage {
    int rows{0};
    int cols{0};
    int top_k{1};
    // Layout: pixel (r, c) slot s -> index r*cols*top_k + c*top_k + s.
    // Slots 0..top_k-1 are kept sorted ascending by range.
    std::vector<float> range;
    std::vector<int>   src_idx;
};

// ---------------------------------------------------------------------------
// Frame (single LiDAR sweep)
// ---------------------------------------------------------------------------

struct ScanFrame {
    std::vector<Point> points_local;
    Pose               T_world_lidar;
    std::int64_t       stamp_ns{0};
};

// ---------------------------------------------------------------------------
// Config — single source of truth (C7)
// ---------------------------------------------------------------------------

struct SensorConfig {
    // Vertical / horizontal field of view, full angle in degrees.
    float vfov_deg{50.0f};
    float hfov_deg{360.0f};
    // Static SE3 offset between body/pose frame and LiDAR frame.
    // Identity by default; KITTI uses Tr (velo->cam0) under the hood, but the
    // dataset reader bakes that into ScanFrame::T_world_lidar already.
    Eigen::Matrix4f T_pose_lidar{Eigen::Matrix4f::Identity()};
};

struct VisibilityConfig {
    // A1: depth peeling. Number of nearest map hits kept per pixel.
    int   top_k{4};
    // A3: log-odds accumulation.
    // Strong static prior so a single transient occluder cannot flip a cell.
    float log_odds_prior{2.0f};
    // Evidence per "ray passed through this map point" carving event.
    float log_odds_carve{-0.4f};
    // Evidence per "this map point matches the scan return".
    float log_odds_hit{0.85f};
    // Decision threshold on accumulated log-odds: cells with ℓ < thr are dynamic.
    float decision_thr{0.0f};
    // Numerical floor/ceiling on accumulated log-odds (clamp).
    float log_odds_min{-8.0f};
    float log_odds_max{ 8.0f};
};

struct RemoveConfig {
    // Coarse-to-fine: each pass uses one resolution. Larger angle = coarser.
    std::vector<float> resolutions_deg{1.0f, 0.5f, 0.25f};
    // A pixel is treated as a free-segment carve target if its scan range is
    // longer than the candidate map range by at least diff_carve_m metres.
    float diff_carve_m{0.5f};
    // A pixel counts as a "hit" if |Δrange| < diff_hit_m.
    float diff_hit_m{0.4f};
    // Fold visibility config inline so RemoveConfig fully describes one pass.
    VisibilityConfig visibility{};
};

struct RevertConfig {
    // Revert is a sign-flipped pass: lower carving evidence, stronger static
    // prior. See plan §P4.
    std::vector<float> resolutions_deg{0.25f, 0.5f, 1.0f};
    float diff_carve_m{0.6f};
    float diff_hit_m{0.4f};
    // Revert lives on the *static* candidate set. We start with a stronger
    // prior toward static and require fewer hits to keep a point.
    VisibilityConfig visibility{
        /*top_k=*/4,
        /*log_odds_prior=*/3.0f,
        /*log_odds_carve=*/-0.2f,
        /*log_odds_hit=*/1.0f,
        /*decision_thr=*/0.0f,
        /*log_odds_min=*/-8.0f,
        /*log_odds_max=*/ 8.0f};
};

struct KnnConfig {
    int   k{6};
    float radius_m{0.5f};
    // Mean scan↔map distance threshold above which a scan-side point is
    // declared dynamic and removed.
    float scan_map_avg_diff_thr{0.10f};
};

struct DownsampleConfig {
    float voxel_size_m{0.05f};
};

struct PipelineConfig {
    SensorConfig     sensor{};
    RemoveConfig     remove{};
    RevertConfig     revert{};
    KnnConfig        knn{};
    DownsampleConfig downsample{};
    int  start_idx{0};
    int  end_idx{-1};        // -1 == until end of dataset
    int  keyframe_gap{1};
    bool use_gpu{false};
    // Output: where the CLI writes static_map.pcd / dynamic_map.pcd / output.h5.
    std::string output_dir{"./removert_out"};
    // Optional .pcd writer toggle (requires PCL at build-time).
    bool write_pcd{true};
    // Optional HDF5 writer toggle (requires HighFive at build-time).
    bool write_hdf5{false};
};

}  // namespace removert
