/**
 * fk_kernel.cu
 * R5.2: templated batch TCP FK kernel. One thread per sample, running frame
 * (3x3 + position) in registers, joint tables staged in shared memory.
 * The fp32 instantiation holds no double arithmetic (checked in SASS, see
 * docs/gpu-precision-decision.md). No fast-math.
 */
#include "fk_kernel.cuh"

#include <cstring>

#include "gpu_manager.cuh"

namespace gpu {

namespace {

__device__ __forceinline__ void sincos_t(float x, float* s, float* c) { sincosf(x, s, c); }
__device__ __forceinline__ void sincos_t(double x, double* s, double* c) { sincos(x, s, c); }

// F = F * G with F = (r[9], p[3]) and G a 3x4 block of the shared table:
// row a, column b of G at off + (a * 4 + b) * stride + j.
template <typename T>
__device__ __forceinline__ void mul_tab(T* r, T* p, const T* tab, int off, int stride, int j) {
    T g[12];
#pragma unroll
    for (int e = 0; e < 12; ++e) g[e] = tab[off + e * stride + j];
    T R[9], P[3];
#pragma unroll
    for (int a = 0; a < 3; ++a) {
#pragma unroll
        for (int c = 0; c < 3; ++c)
            R[a * 3 + c] = r[a * 3 + 0] * g[0 * 4 + c] + r[a * 3 + 1] * g[1 * 4 + c] +
                           r[a * 3 + 2] * g[2 * 4 + c];
        P[a] = r[a * 3 + 0] * g[3] + r[a * 3 + 1] * g[7] + r[a * 3 + 2] * g[11] + p[a];
    }
#pragma unroll
    for (int e = 0; e < 9; ++e) r[e] = R[e];
#pragma unroll
    for (int a = 0; a < 3; ++a) p[a] = P[a];
}

template <typename T>
__global__ void __launch_bounds__(FK_THREADS)
fk_tcp_kernel(int n, const T* __restrict__ tab, const int* __restrict__ itab,
              const T* __restrict__ q, long long ld, T* __restrict__ out, long long ldo,
              long long count) {
    __shared__ T s_tab[FK_MAX_JOINTS * FK_TAB_PER_JOINT + 12];
    __shared__ int s_itab[2 * FK_MAX_JOINTS];
    const int nt = n * FK_TAB_PER_JOINT + 12;
    for (int k = threadIdx.x; k < nt; k += blockDim.x) s_tab[k] = tab[k];
    for (int k = threadIdx.x; k < 2 * n; k += blockDim.x) s_itab[k] = itab[k];
    __syncthreads();

    const int off_post = 12 * n, off_axis = 24 * n, off_mult = 27 * n, off_offs = 28 * n,
              off_tool = 29 * n;
    const long long stride = static_cast<long long>(gridDim.x) * blockDim.x;
    for (long long i = static_cast<long long>(blockIdx.x) * blockDim.x + threadIdx.x; i < count;
         i += stride) {
        T r[9] = {T(1), T(0), T(0), T(0), T(1), T(0), T(0), T(0), T(1)};
        T p[3] = {T(0), T(0), T(0)};
        for (int j = 0; j < n; ++j) {
            mul_tab<T>(r, p, s_tab, 0, n, j);  // pre (fixed joints: folded pre * post)
            const int ty = s_itab[j];
            if (ty == 2) continue;
            const T qe = s_tab[off_mult + j] * q[static_cast<long long>(s_itab[n + j]) * ld + i] +
                         s_tab[off_offs + j];
            const T ax = s_tab[off_axis + 0 * n + j];
            const T ay = s_tab[off_axis + 1 * n + j];
            const T az = s_tab[off_axis + 2 * n + j];
            if (ty == 0) {
                T s, c;
                sincos_t(qe, &s, &c);
                const T t = T(1) - c;
                const T m[9] = {c + t * ax * ax,      t * ax * ay - s * az, t * ax * az + s * ay,
                                t * ax * ay + s * az, c + t * ay * ay,      t * ay * az - s * ax,
                                t * ax * az - s * ay, t * ay * az + s * ax, c + t * az * az};
                T nr[9];
#pragma unroll
                for (int a = 0; a < 3; ++a)
#pragma unroll
                    for (int b = 0; b < 3; ++b)
                        nr[a * 3 + b] = r[a * 3 + 0] * m[0 * 3 + b] + r[a * 3 + 1] * m[1 * 3 + b] +
                                        r[a * 3 + 2] * m[2 * 3 + b];
#pragma unroll
                for (int e = 0; e < 9; ++e) r[e] = nr[e];
            } else {
                const T d0 = ax * qe, d1 = ay * qe, d2 = az * qe;
#pragma unroll
                for (int a = 0; a < 3; ++a)
                    p[a] += r[a * 3 + 0] * d0 + r[a * 3 + 1] * d1 + r[a * 3 + 2] * d2;
            }
            mul_tab<T>(r, p, s_tab, off_post, n, j);
        }
        mul_tab<T>(r, p, s_tab, off_tool, 1, 0);  // tool: element e at off_tool + e
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
