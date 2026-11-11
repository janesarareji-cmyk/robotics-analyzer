/**
 * fk_kernel.cu
 * R5.2: templated batch TCP FK kernel. One thread per sample, running frame
 * (3x3 + position) in registers, joint tables staged in shared memory.
 * The fp32 instantiation holds no double arithmetic (checked in SASS, see
 * docs/gpu-precision-decision.md). No fast-math.
 */
#include "fk_kernel.cuh"
#include "fk_device.cuh"

#include <cstring>

#include "gpu_manager.cuh"

namespace gpu {

namespace {

template <typename T>
struct GlobalQ {
    const T* q;
    long long ld, i;
    __device__ __forceinline__ T operator()(int src) const { return q[src * ld + i]; }
};

template <typename T>
__global__ void __launch_bounds__(FK_THREADS)
fk_tcp_kernel(int n, const T* __restrict__ tab, const int* __restrict__ itab,
              const T* __restrict__ q, long long ld, T* __restrict__ out, long long ldo,
              long long count) {
    __shared__ T s_tab[FK_MAX_JOINTS * FK_TAB_PER_JOINT + 12];
    __shared__ int s_itab[2 * FK_MAX_JOINTS];
    fkdev::stage_tables<T>(n, tab, itab, s_tab, s_itab);

    const long long stride = static_cast<long long>(gridDim.x) * blockDim.x;
    for (long long i = static_cast<long long>(blockIdx.x) * blockDim.x + threadIdx.x; i < count;
         i += stride) {
        T r[9], p[3];
        fkdev::tcp_walk<T>(n, s_tab, s_itab, GlobalQ<T>{q, ld, i}, r, p);
        out[0 * ldo + i] = p[0];
        out[1 * ldo + i] = p[1];
        out[2 * ldo + i] = p[2];
#pragma unroll
        for (int e = 0; e < 9; ++e) out[(3 + e) * ldo + i] = r[e];
    }
}

// 3x4 row-major product A * B (last row 0 0 0 1 implied).
void mul34(const double* A, const double* B, double* C) {
    for (int a = 0; a < 3; ++a) {
        for (int c = 0; c < 4; ++c) {
            double v = 0.0;
            for (int k = 0; k < 3; ++k) v += A[a * 4 + k] * B[k * 4 + c];
            if (c == 3) v += A[a * 4 + 3];
            C[a * 4 + c] = v;
        }
    }
}

template <typename T>
void pack(const FkPathTables& t, std::vector<char>& tab) {
    const size_t n = static_cast<size_t>(t.n);
    std::vector<T> v(n * FK_TAB_PER_JOINT + 12, T(0));
    const double I34[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    for (size_t j = 0; j < n; ++j) {
        double pre[12], post[12];
        std::memcpy(pre, &t.pre[j * 12], sizeof pre);
        std::memcpy(post, &t.post[j * 12], sizeof post);
        if (t.type[j] == 2) {
            double f[12];
            mul34(pre, post, f);
            std::memcpy(pre, f, sizeof f);
            std::memcpy(post, I34, sizeof post);
        }
        for (size_t e = 0; e < 12; ++e) {
            v[e * n + j] = static_cast<T>(pre[e]);
            v[(12 + e) * n + j] = static_cast<T>(post[e]);
        }
        for (size_t e = 0; e < 3; ++e) v[(24 + e) * n + j] = static_cast<T>(t.axis[j * 3 + e]);
        v[27 * n + j] = static_cast<T>(t.mult[j]);
        v[28 * n + j] = static_cast<T>(t.offset[j]);
    }
    for (size_t e = 0; e < 12; ++e) v[29 * n + e] = static_cast<T>(t.tool[e]);
    tab.resize(v.size() * sizeof(T));
    std::memcpy(tab.data(), v.data(), tab.size());
}

}  // namespace

size_t fk_tab_bytes(int n, bool fp64) {
    return (static_cast<size_t>(n) * FK_TAB_PER_JOINT + 12) *
           (fp64 ? sizeof(double) : sizeof(float));
}

size_t fk_itab_bytes(int n) { return static_cast<size_t>(2 * n) * sizeof(int); }

void fk_pack_tables(const FkPathTables& t, bool fp64, std::vector<char>& tab,
                    std::vector<int>& itab) {
    if (fp64) pack<double>(t, tab); else pack<float>(t, tab);
    itab.assign(static_cast<size_t>(2 * t.n), 0);
    for (int j = 0; j < t.n; ++j) {
        itab[j] = t.type[j];
        itab[t.n + j] = t.src_q[j] < 0 ? 0 : t.src_q[j];
    }
}

void fk_launch(bool fp64, int n, const void* d_tab, const int* d_itab, const void* d_q,
               long long ld, void* d_out, long long ldo, long long count, int grid,
               cudaStream_t stream) {
    if (count <= 0) return;
    if (fp64) {
        fk_tcp_kernel<double><<<grid, FK_THREADS, 0, stream>>>(
            n, static_cast<const double*>(d_tab), d_itab, static_cast<const double*>(d_q), ld,
            static_cast<double*>(d_out), ldo, count);
    } else {
        fk_tcp_kernel<float><<<grid, FK_THREADS, 0, stream>>>(
            n, static_cast<const float*>(d_tab), d_itab, static_cast<const float*>(d_q), ld,
            static_cast<float*>(d_out), ldo, count);
    }
    CUDA_CHECK(cudaGetLastError());
#ifndef NDEBUG
    CUDA_CHECK(cudaStreamSynchronize(stream));
#endif
}

}  // namespace gpu
