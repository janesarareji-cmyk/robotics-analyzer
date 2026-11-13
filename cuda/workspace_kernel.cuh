/**
 * workspace_kernel.cuh
 * R5.3: workspace sampling kernel (fp32 only). q is generated on the device with
 * Philox (no q buffer), the TCP is walked in the BASE frame, the position is
 * voxelized into an occupied bitset (atomicOr) and optional counts (atomicAdd).
 * Plain C++ interface, no Eigen.
 */
#pragma once

#include <cuda_runtime.h>

#include <cstddef>
#include <cstdint>

namespace gpu {

struct WsKernelArgs {
    int n = 0;                       ///< joints on the active path
    const float* d_tab = nullptr;    ///< fk_pack_tables(fp64 = false)
    const int* d_itab = nullptr;
    int n_dof = 0;
    const float* d_lo = nullptr;     ///< per-DOF sampler constants (SamplerTables)
    const float* d_a = nullptr;
    const int* d_div = nullptr;
    bool grid_mode = false;
    unsigned long long seed = 42;
    long long start = 0;             ///< global index of the first sample of the chunk
    long long count = 0;             ///< samples in the chunk
    float origin[3] = {0, 0, 0};
    float voxel = 1.0f;
    int dims[3] = {1, 1, 1};
    unsigned int* d_occ = nullptr;      ///< bitset words
    unsigned int* d_counts = nullptr;   ///< optional
    unsigned long long* d_outside = nullptr;
    float* d_points = nullptr;       ///< optional (pt_count, 3)
    long long pt_count = 0;          ///< the first pt_count samples of the chunk write a point
};

/// Launch the workspace kernel. Throws gpu::CudaError on launch failure.
void ws_launch(const WsKernelArgs& a, int grid, cudaStream_t stream);

/// Debug kernel: q (count, n_dof) float row-major for global samples start..start+count.
void ws_gen_q(int n_dof, const float* d_lo, const float* d_a, const int* d_div, bool grid_mode,
              unsigned long long seed, long long start, long long count, float* d_q, int grid,
              cudaStream_t stream);

/// Number of set bits in a bitset of n_words words (small internal temp allocation).
/// Counts set bits. Returns false (out = 0) when its scratch allocation hits out-of-memory
/// (caller takes the host path); other CUDA errors throw (A1.5).
bool ws_count_bits(const unsigned int* d_occ, size_t n_words, cudaStream_t stream,
                   unsigned long long& out);

/// cub::DeviceSelect::Flagged over voxel linear indices 0..n_voxels (flag = bit set).
/// Pass d_temp = nullptr to query temp bytes. d_out holds n_set uint32 linear indices.
size_t ws_compact(void* d_temp, size_t temp_bytes, const unsigned int* d_occ,
                  unsigned int n_voxels, unsigned int* d_out, int* d_num_selected,
                  cudaStream_t stream);

}  // namespace gpu
