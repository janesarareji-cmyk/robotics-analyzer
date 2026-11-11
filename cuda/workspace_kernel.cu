/**
 * workspace_kernel.cu
 * R5.3: workspace sampling kernel (fp32), debug q generator, bit count and
 * cub::DeviceSelect::Flagged compaction of the occupied voxel indices.
 * The fp32 TCP walk is the R5.2 code (fk_device.cuh). No fast-math.
 */
#include "workspace_kernel.cuh"

#include <cub/cub.cuh>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/transform_iterator.h>

#include "compute/philox.hpp"
#include "fk_device.cuh"
#include "gpu_manager.cuh"

namespace gpu {

namespace {

constexpr int kMaxDof = FK_MAX_JOINTS;

struct LocalQ {
    const float* q;
    __device__ __forceinline__ float operator()(int src) const { return q[src]; }
};

__global__ void __launch_bounds__(FK_THREADS) ws_kernel(WsKernelArgs a) {
    __shared__ float s_tab[FK_MAX_JOINTS * FK_TAB_PER_JOINT + 12];
    __shared__ int s_itab[2 * FK_MAX_JOINTS];
    __shared__ float s_lo[kMaxDof], s_a[kMaxDof];
    __shared__ int s_div[kMaxDof];
    fkdev::stage_tables<float>(a.n, a.d_tab, a.d_itab, s_tab, s_itab);
    for (int k = threadIdx.x; k < a.n_dof; k += blockDim.x) {
        s_lo[k] = a.d_lo[k];
        s_a[k] = a.d_a[k];
        s_div[k] = a.d_div[k];
    }
    __syncthreads();

    const long long stride = static_cast<long long>(gridDim.x) * blockDim.x;
    unsigned long long outside = 0;
    const unsigned int nx = static_cast<unsigned int>(a.dims[0]);
    const unsigned int ny = static_cast<unsigned int>(a.dims[1]);
    const unsigned int nz = static_cast<unsigned int>(a.dims[2]);
    for (long long i = static_cast<long long>(blockIdx.x) * blockDim.x + threadIdx.x; i < a.count;
         i += stride) {
        float q[kMaxDof];
        const unsigned long long gi = static_cast<unsigned long long>(a.start + i);
        if (a.grid_mode)
            compute::fill_q_grid(q, a.n_dof, gi, s_lo, s_a, s_div);
        else
            compute::fill_q_random(q, a.n_dof, a.seed, gi, s_lo, s_a);
        float r[9], p[3];
        fkdev::tcp_walk<float>(a.n, s_tab, s_itab, LocalQ{q}, r, p);
        if (i < a.pt_count) {
            a.d_points[i * 3 + 0] = p[0];
            a.d_points[i * 3 + 1] = p[1];
            a.d_points[i * 3 + 2] = p[2];
        }
        const float fx = floorf((p[0] - a.origin[0]) / a.voxel);
        const float fy = floorf((p[1] - a.origin[1]) / a.voxel);
        const float fz = floorf((p[2] - a.origin[2]) / a.voxel);
        if (!(fx >= 0.0f && fx < static_cast<float>(nx) && fy >= 0.0f &&
              fy < static_cast<float>(ny) && fz >= 0.0f && fz < static_cast<float>(nz))) {
            ++outside;
            continue;
        }
        const unsigned int lin = (static_cast<unsigned int>(fx) * ny + static_cast<unsigned int>(fy)) *
                                     nz + static_cast<unsigned int>(fz);
        const unsigned int bit = 1u << (lin & 31u);
        unsigned int* w = a.d_occ + (lin >> 5);
        if (!(__ldg(w) & bit)) atomicOr(w, bit);
        if (a.d_counts) atomicAdd(a.d_counts + lin, 1u);
    }
    if (outside) atomicAdd(a.d_outside, outside);
}

__global__ void ws_gen_q_kernel(int n_dof, const float* lo, const float* aa, const int* dv,
                                bool grid_mode, unsigned long long seed, long long start,
                                long long count, float* out) {
    const long long stride = static_cast<long long>(gridDim.x) * blockDim.x;
    for (long long i = static_cast<long long>(blockIdx.x) * blockDim.x + threadIdx.x; i < count;
         i += stride) {
        float q[kMaxDof];
        const unsigned long long gi = static_cast<unsigned long long>(start + i);
        if (grid_mode)
            compute::fill_q_grid(q, n_dof, gi, lo, aa, dv);
        else
            compute::fill_q_random(q, n_dof, seed, gi, lo, aa);
        for (int d = 0; d < n_dof; ++d) out[i * n_dof + d] = q[d];
    }
}

struct PopCount {
    __host__ __device__ unsigned long long operator()(unsigned int w) const {
#if defined(__CUDA_ARCH__)
        return static_cast<unsigned long long>(__popc(w));
#else
        unsigned long long c = 0;
        for (; w; w &= w - 1) ++c;
        return c;
#endif
    }
};

struct BitFlag {
    const unsigned int* occ;
    __host__ __device__ bool operator()(unsigned int lin) const {
        return (occ[lin >> 5] >> (lin & 31u)) & 1u;
    }
};

}  // namespace

void ws_launch(const WsKernelArgs& a, int grid, cudaStream_t stream) {
    if (a.count <= 0) return;
    ws_kernel<<<grid, FK_THREADS, 0, stream>>>(a);
    CUDA_CHECK(cudaGetLastError());
#ifndef NDEBUG
    CUDA_CHECK(cudaStreamSynchronize(stream));
#endif
}

void ws_gen_q(int n_dof, const float* d_lo, const float* d_a, const int* d_div, bool grid_mode,
              unsigned long long seed, long long start, long long count, float* d_q, int grid,
              cudaStream_t stream) {
    if (count <= 0) return;
    ws_gen_q_kernel<<<grid, FK_THREADS, 0, stream>>>(n_dof, d_lo, d_a, d_div, grid_mode, seed,
                                                     start, count, d_q);
    CUDA_CHECK(cudaGetLastError());
}

unsigned long long ws_count_bits(const unsigned int* d_occ, size_t n_words, cudaStream_t stream) {
    if (n_words == 0) return 0;
    auto it = thrust::make_transform_iterator(d_occ, PopCount{});
    unsigned long long* d_res = nullptr;
    CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_res), sizeof(unsigned long long)));
    void* d_temp = nullptr;
    size_t temp = 0;
    cudaError_t e = cub::DeviceReduce::Sum(d_temp, temp, it, d_res, static_cast<int>(n_words), stream);
    if (e == cudaSuccess) e = cudaMalloc(&d_temp, temp ? temp : 1);
    if (e == cudaSuccess) e = cub::DeviceReduce::Sum(d_temp, temp, it, d_res, static_cast<int>(n_words), stream);
    unsigned long long h = 0;
    if (e == cudaSuccess) e = cudaMemcpyAsync(&h, d_res, sizeof h, cudaMemcpyDeviceToHost, stream);
    if (e == cudaSuccess) e = cudaStreamSynchronize(stream);
    if (d_temp) cudaFree(d_temp);
    cudaFree(d_res);
    CUDA_CHECK(e);
    return h;
}

size_t ws_compact(void* d_temp, size_t temp_bytes, const unsigned int* d_occ, unsigned int n_voxels,
                  unsigned int* d_out, int* d_num_selected, cudaStream_t stream) {
    thrust::counting_iterator<unsigned int> idx(0);
    auto flags = thrust::make_transform_iterator(idx, BitFlag{d_occ});
    size_t need = temp_bytes;
    CUDA_CHECK(cub::DeviceSelect::Flagged(d_temp, need, idx, flags, d_out, d_num_selected,
                                          static_cast<int>(n_voxels), stream));
    return need;
}

}  // namespace gpu
