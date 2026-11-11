/**
 * ik_gpu.hpp
 * R5.4 part B: host driver for the GPU IK prototype (GPU_IK_EXPERIMENTAL).
 * Chunking, pack/unpack, OOM halving. No pybind11, no Eigen.
 */
#pragma once

#include <string>
#include <vector>

#include "fk_kernel.cuh"
#include "ik_kernel.cuh"

namespace gpu {

struct IkGpuIn {
    const FkPathTables* tables = nullptr;
    IkParams prm;                    ///< nd, weights, tolerances, limits, caps
    const double* targets = nullptr; ///< N row-major 4x4 (world frame)
    const double* q0 = nullptr;      ///< N * nd, or nd when q0_broadcast
    bool q0_broadcast = false;
    std::size_t N = 0;
    double base[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};  ///< row-major
    long long forced_chunk = 0;      ///< test hook (multiple of 128); 0 = automatic
    int n_threads = 0;
};

struct IkGpuOut {
    std::vector<double> q;     ///< N * nd (float results widened)
    std::vector<int> status;   ///< IKS_*
    std::vector<int> iters;
    std::vector<float> perr, rerr;
    int chunks = 0;
    long long chunk = 0;
    double kernel_ms = 0.0;
};

/// true: GPU did the work. false: `reason` says why (caller falls back to the CPU).
/// Throws gpu::CudaError on non-OOM CUDA failures (sticky ones mark the GPU dead).
bool ik_gpu_run(const IkGpuIn& in, IkGpuOut& out, std::string& reason);

}  // namespace gpu
