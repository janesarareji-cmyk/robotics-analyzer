/**
 * fk_device.cuh
 * R5.2/R5.3: device-side TCP FK pieces shared by the batch FK kernel and the
 * workspace kernel: shared-memory table staging and the per-sample path walk.
 * Table layout: see fk_kernel.cuh (SoA, element e of joint j at e * n + j).
 */
#pragma once

#include <cuda_runtime.h>

#include "fk_kernel.cuh"

namespace gpu {
namespace fkdev {

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


/// Stage the value table and index table into shared memory. All threads of the
/// block must call it (contains __syncthreads).
template <typename T>
__device__ __forceinline__ void stage_tables(int n, const T* __restrict__ tab,
                                             const int* __restrict__ itab, T* s_tab, int* s_itab) {
    const int nt = n * FK_TAB_PER_JOINT + 12;
    for (int k = threadIdx.x; k < nt; k += blockDim.x) s_tab[k] = tab[k];
    for (int k = threadIdx.x; k < 2 * n; k += blockDim.x) s_itab[k] = itab[k];
    __syncthreads();
}

/// TCP (r[9] row-major, p[3]) in the BASE frame; qsrc(src_dof) returns the dof value.
template <typename T, class QSrc>
__device__ __forceinline__ void tcp_walk(int n, const T* s_tab, const int* s_itab, const QSrc& qsrc,
                                         T* r, T* p) {
    const int off_post = 12 * n, off_axis = 24 * n, off_mult = 27 * n, off_offs = 28 * n,
              off_tool = 29 * n;
    r[0] = T(1); r[1] = T(0); r[2] = T(0); r[3] = T(0); r[4] = T(1); r[5] = T(0);
    r[6] = T(0); r[7] = T(0); r[8] = T(1);
    p[0] = T(0); p[1] = T(0); p[2] = T(0);
    for (int j = 0; j < n; ++j) {
        mul_tab<T>(r, p, s_tab, 0, n, j);  // pre (fixed joints: folded pre * post)
        const int ty = s_itab[j];
        if (ty == 2) continue;
        const T qe = s_tab[off_mult + j] * qsrc(s_itab[n + j]) + s_tab[off_offs + j];
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
}

}  // namespace fkdev
}  // namespace gpu
