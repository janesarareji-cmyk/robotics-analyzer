/**
 * workspace.hpp
 * R5.3: workspace sampling. Shared types (options, grid, result), validation,
 * the sampler tables shared with the GPU, and the multi-threaded CPU reference.
 *
 * Sampled TCP positions are in the BASE frame (the model base is never applied
 * here, R5.2 rule). Voxel rule: index = floor((p - origin) / voxel) per axis in
 * the arithmetic of the engine (double on the CPU, float on the GPU); a point
 * outside [0, dims) is counted in n_outside and dropped.
 * Linear voxel index: lin = (ix * ny + iy) * nz + iz (C order of an (nx,ny,nz) array).
 * Occupied bitset: uint32 words, bit (lin & 31) of word (lin >> 5).
 */
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "kin/compiled_model.hpp"

namespace compute {

constexpr std::size_t kMaxOccupiedVoxels = 1024ull * 1024ull * 1024ull;  // bitset cap: 128 MB
constexpr std::size_t kMaxCountsBytes = 256ull * 1024ull * 1024ull;       // counts cap

struct WorkspaceOptions {
    std::string mode = "random";          ///< "random" or "grid"
    std::size_t n_samples = 0;            ///< random: samples; grid: cap on the product
    std::uint64_t seed = 42;
    double voxel_size = 0.0;
    bool bounds_set = false;              ///< false: [-R, R]^3 + 1 voxel (R = reach bound)
    double bounds_lo[3] = {0, 0, 0};
    double bounds_hi[3] = {0, 0, 0};
    bool want_counts = true;
    bool want_points = false;
    std::size_t max_points = 1000000;
    std::size_t budget_bytes = 0;         ///< 0 = auto from free VRAM (GPU only)
    std::vector<int> divisions;           ///< grid mode: per-DOF divisions
    long long chunk = 0;                  ///< test hook: forced chunk (multiple of 256)
    int n_threads = 0;                    ///< CPU threads
};

struct WorkspaceGrid {
    int dims[3] = {0, 0, 0};
    double origin[3] = {0, 0, 0};
    double voxel = 0.0;
    std::size_t n_voxels = 0;
};

/// Per-DOF float sampler constants (identical on host and device).
struct SamplerTables {
    bool grid = false;
    std::uint64_t seed = 42;
    int n_dof = 0;
    std::vector<float> lo;     ///< random: lo_f; grid: lo_f (or the center when div == 1)
    std::vector<float> a;      ///< random: hi_f - lo_f; grid: step
    std::vector<int> div;      ///< grid divisions (1 in random mode)
};

struct WorkspaceResult {
    WorkspaceGrid grid;
    std::vector<std::uint32_t> occupied;  ///< bitset words
    std::vector<std::uint32_t> counts;    ///< n_voxels (empty if not wanted)
    std::vector<std::int32_t> voxels;     ///< (M,3) compacted occupied voxel indices
    std::vector<float> points;            ///< (P,3) first P samples (empty if not wanted)
    std::size_t n_samples = 0;            ///< total samples of the run
    std::size_t n_done = 0;
    std::size_t n_outside = 0;
    bool cancelled = false;
    // info
    std::string backend = "cpu";
    std::string fallback_reason;
    int chunks = 0;
    long long chunk = 0;
    std::size_t vram_peak_bytes = 0;
    std::size_t budget_bytes = 0;
    double kernel_ms = 0.0;
    double total_ms = 0.0;
    bool gpu_compaction = false;
};

using ProgressFn = std::function<void(std::size_t done, std::size_t total)>;

/// Upper bound of |TCP position| in the base frame (sum of translation norms on the
/// active path plus the prismatic travel for the given DOF limits).
double reach_bound(const kin::CompiledModel& cm, const std::vector<double>& lo,
                   const std::vector<double>& hi);

/// Validates inputs and returns the grid. Throws std::invalid_argument (-> ValueError)
/// on bad inputs or when the grid exceeds the caps.
WorkspaceGrid resolve_grid(const kin::CompiledModel& cm, const std::vector<double>& lo,
                           const std::vector<double>& hi, const WorkspaceOptions& opt);

/// Total number of samples (random: n_samples; grid: the product of divisions).
std::size_t total_samples(const WorkspaceOptions& opt, int n_dof);

SamplerTables make_sampler(const std::vector<double>& lo, const std::vector<double>& hi,
                           const WorkspaceOptions& opt);

/// q for global samples [start, start + n) as float, row-major (n, n_dof).
void sample_q_host(const SamplerTables& s, std::size_t start, std::size_t n, float* out);

/// Host extraction of occupied voxel indices from the bitset (ascending lin order), (M,3).
void bitset_to_voxels(const std::uint32_t* words, const WorkspaceGrid& g,
                      std::vector<std::int32_t>& out);

/// CPU reference (double FK on the float q of the sampler). cancel / progress may be empty.
WorkspaceResult sample_workspace_cpu(const kin::CompiledModel& cm, const std::vector<double>& lo,
                                     const std::vector<double>& hi, const WorkspaceOptions& opt,
                                     const std::atomic<bool>* cancel = nullptr,
                                     const ProgressFn& progress = nullptr);

}  // namespace compute
