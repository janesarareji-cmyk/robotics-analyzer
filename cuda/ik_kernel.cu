/**
 * ik_kernel.cu
 * R5.4 part B: fp32 DLS IK prototype kernel (GPU_IK_EXPERIMENTAL, not used by the
 * backend). One thread per target, base frame. Per iteration: FK of the trial q (shared
 * path walk from fk_device.cuh), geometric Jacobian J[6][NMAX] in registers (rebuilt only
 * after an accepted step), A = J J^T (21 unique entries) + lambda^2 I, unrolled 6x6
 * Cholesky, dq = J^T y. No fast-math, no double arithmetic.
 *
 * Rules (R5 spec, R5.4 item 2): lambda_min 5e-3; pivot guard (pivot <= 1e-12 * trace ->
 * lambda *= 10, retry); step clamp 0.2 rad / 0.05 m (uniform scaling of dq, keeps the
 * direction), then joint-limit clamp; error up -> reject the step and lambda *= 10,
 * error down -> accept and lambda /= 2; 100 iteration cap; stall = error reduced by less
 * than 10 % over 10 iterations; non-finite input or error -> IKS_NAN; a thread that is
 * done leaves the loop (done mask), launch 128 threads/block.
 */
#include "ik_kernel.cuh"

#include "fk_device.cuh"
#include "gpu_manager.cuh"

namespace gpu {

namespace {

template <int NMAX>
struct QReg {
    const float (&q)[NMAX];
    __device__ __forceinline__ float operator()(int s) const {
        float v = 0.f;
#pragma unroll
        for (int d = 0; d < NMAX; ++d)
            if (d == s) v = q[d];
        return v;
    }
};

/// J (6 x NMAX) of the TCP in the base frame. ptcp = TCP position (base frame) at q.
template <int NMAX>
__device__ __forceinline__ void jac_walk(int n, const float* s_tab, const int* s_itab,
                                         const float (&q)[NMAX], const float* ptcp,
                                         float (&J)[6][NMAX]) {
    const int off_post = 12 * n, off_axis = 24 * n, off_mult = 27 * n, off_offs = 28 * n;
#pragma unroll
    for (int a = 0; a < 6; ++a)
#pragma unroll
        for (int d = 0; d < NMAX; ++d) J[a][d] = 0.f;
    float r[9] = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f};
    float p[3] = {0.f, 0.f, 0.f};
    const QReg<NMAX> qs{q};
    for (int j = 0; j < n; ++j) {
        fkdev::mul_tab<float>(r, p, s_tab, 0, n, j);
        const int ty = s_itab[j];
        if (ty == 2) continue;
        const int src = s_itab[n + j];
        const float mult = s_tab[off_mult + j];
        const float qe = mult * qs(src) + s_tab[off_offs + j];
        const float ax = s_tab[off_axis + 0 * n + j];
        const float ay = s_tab[off_axis + 1 * n + j];
        const float az = s_tab[off_axis + 2 * n + j];
        float aw[3];
#pragma unroll
        for (int a = 0; a < 3; ++a) aw[a] = r[a * 3 + 0] * ax + r[a * 3 + 1] * ay + r[a * 3 + 2] * az;
        float col[6];
        if (ty == 0) {
            const float d0 = ptcp[0] - p[0], d1 = ptcp[1] - p[1], d2 = ptcp[2] - p[2];
            col[0] = aw[1] * d2 - aw[2] * d1;
            col[1] = aw[2] * d0 - aw[0] * d2;
            col[2] = aw[0] * d1 - aw[1] * d0;
            col[3] = aw[0];
            col[4] = aw[1];
            col[5] = aw[2];
            float s, c;
            fkdev::sincos_t(qe, &s, &c);
            const float t = 1.f - c;
            const float m[9] = {c + t * ax * ax,      t * ax * ay - s * az, t * ax * az + s * ay,
                                t * ax * ay + s * az, c + t * ay * ay,      t * ay * az - s * ax,
                                t * ax * az - s * ay, t * ay * az + s * ax, c + t * az * az};
            float nr[9];
#pragma unroll
            for (int a = 0; a < 3; ++a)
#pragma unroll
                for (int b = 0; b < 3; ++b)
                    nr[a * 3 + b] = r[a * 3 + 0] * m[0 * 3 + b] + r[a * 3 + 1] * m[1 * 3 + b] +
                                    r[a * 3 + 2] * m[2 * 3 + b];
#pragma unroll
            for (int e = 0; e < 9; ++e) r[e] = nr[e];
        } else {
            col[0] = aw[0];
            col[1] = aw[1];
            col[2] = aw[2];
            col[3] = col[4] = col[5] = 0.f;
            const float d0 = ax * qe, d1 = ay * qe, d2 = az * qe;
#pragma unroll
            for (int a = 0; a < 3; ++a)
                p[a] += r[a * 3 + 0] * d0 + r[a * 3 + 1] * d1 + r[a * 3 + 2] * d2;
        }
#pragma unroll
        for (int d = 0; d < NMAX; ++d)
            if (d == src) {
#pragma unroll
                for (int a = 0; a < 6; ++a) J[a][d] += mult * col[a];
            }
        fkdev::mul_tab<float>(r, p, s_tab, off_post, n, j);
    }
}

/// e = [target - tcp position ; log(Rt * R^T)] (base frame).
__device__ __forceinline__ void pose_err(const float* tr, const float* tp, const float* r,
                                         const float* p, float* e) {
    e[0] = tp[0] - p[0];
    e[1] = tp[1] - p[1];
    e[2] = tp[2] - p[2];
    float R[3][3];  // Rerr = Rt * R^T
#pragma unroll
    for (int a = 0; a < 3; ++a)
#pragma unroll
        for (int b = 0; b < 3; ++b)
            R[a][b] = tr[a * 3 + 0] * r[b * 3 + 0] + tr[a * 3 + 1] * r[b * 3 + 1] +
                      tr[a * 3 + 2] * r[b * 3 + 2];
    const float vx = 0.5f * (R[2][1] - R[1][2]);
    const float vy = 0.5f * (R[0][2] - R[2][0]);
    const float vz = 0.5f * (R[1][0] - R[0][1]);
    const float s = sqrtf(vx * vx + vy * vy + vz * vz);  // sin(theta)
    const float c = 0.5f * (R[0][0] + R[1][1] + R[2][2] - 1.f);
    const float theta = atan2f(s, c);
    if (s < 1e-4f && c < 0.f) {  // theta near pi: axis from the diagonal
        int m = 0;
        if (R[1][1] > R[m][m]) m = 1;
        if (R[2][2] > R[m][m]) m = 2;
        float ax[3];
        ax[m] = sqrtf(fmaxf(0.5f * (R[m][m] + 1.f), 1e-12f));
#pragma unroll
        for (int i = 0; i < 3; ++i)
            if (i != m) ax[i] = (R[m][i] + R[i][m]) / (4.f * ax[m]);
        const float nrm = sqrtf(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
        const float k = theta / fmaxf(nrm, 1e-12f);
        e[3] = ax[0] * k;
        e[4] = ax[1] * k;
        e[5] = ax[2] * k;
    } else {
        const float k = (s < 1e-6f) ? 1.f : theta / s;
        e[3] = vx * k;
        e[4] = vy * k;
        e[5] = vz * k;
    }
}

/// Solve (A + lam2 I) y = b, A symmetric (lower triangle used). false: pivot guard hit.
__device__ __forceinline__ bool chol_solve6(const float (&A)[6][6], float lam2, const float (&b)[6],
                                            float (&y)[6]) {
    float tr = 0.f;
#pragma unroll
    for (int i = 0; i < 6; ++i) tr += A[i][i] + lam2;
    const float thr = 1e-12f * tr;
    float L[6][6];
#pragma unroll
    for (int i = 0; i < 6; ++i) {
#pragma unroll
        for (int j = 0; j <= i; ++j) {
            float s = A[i][j] + ((i == j) ? lam2 : 0.f);
#pragma unroll
            for (int k = 0; k < j; ++k) s -= L[i][k] * L[j][k];
            if (i == j) {
                if (!(s > thr) || !isfinite(s)) return false;
                L[i][i] = sqrtf(s);
            } else {
                L[i][j] = s / L[j][j];
            }
        }
    }
    float z[6];
#pragma unroll
    for (int i = 0; i < 6; ++i) {
        float s = b[i];
#pragma unroll
        for (int k = 0; k < i; ++k) s -= L[i][k] * z[k];
        z[i] = s / L[i][i];
    }
#pragma unroll
    for (int i = 5; i >= 0; --i) {
        float s = z[i];
#pragma unroll
        for (int k = i + 1; k < 6; ++k) s -= L[k][i] * y[k];
        y[i] = s / L[i][i];
    }
    return true;
}

template <int NMAX>
__global__ void __launch_bounds__(IK_THREADS)
ik_kernel(int n, const float* __restrict__ tab, const int* __restrict__ itab, const IkParams P,
          const float* __restrict__ tgt, const float* __restrict__ q0, long long ld,
          float* __restrict__ qout, int* __restrict__ st, float* __restrict__ perr,
          float* __restrict__ rerr, int* __restrict__ iters, long long count) {
    __shared__ float s_tab[FK_MAX_JOINTS * FK_TAB_PER_JOINT + 12];
    __shared__ int s_itab[2 * FK_MAX_JOINTS];
    fkdev::stage_tables<float>(n, tab, itab, s_tab, s_itab);

    const long long i = static_cast<long long>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (i >= count) return;
    const int nd = P.nd;

    float tp[3], tr[9];
    for (int k = 0; k < 3; ++k) tp[k] = tgt[k * ld + i];
    for (int k = 0; k < 9; ++k) tr[k] = tgt[(3 + k) * ld + i];
    float q[NMAX];
    bool finite = true;
    for (int k = 0; k < 3; ++k) finite = finite && isfinite(tp[k]);
    for (int k = 0; k < 9; ++k) finite = finite && isfinite(tr[k]);
#pragma unroll
    for (int d = 0; d < NMAX; ++d) {
        q[d] = (d < nd) ? q0[d * ld + i] : 0.f;
        finite = finite && isfinite(q[d]);
    }
    if (!finite) {
#pragma unroll
        for (int d = 0; d < NMAX; ++d)
            if (d < nd) qout[d * ld + i] = q[d];
        st[i] = IKS_NAN;
        perr[i] = 0.f;
        rerr[i] = 0.f;
        iters[i] = 0;
        return;
    }
#pragma unroll
    for (int d = 0; d < NMAX; ++d)
        if (d < nd) q[d] = fminf(fmaxf(q[d], P.lo[d]), P.hi[d]);

    auto weighted = [&](const float* e, float& wp, float& wr) {
        const float a = P.w[0] * e[0], b = P.w[1] * e[1], c = P.w[2] * e[2];
        const float x = P.w[3] * e[3], y = P.w[4] * e[4], z = P.w[5] * e[5];
        wp = sqrtf(a * a + b * b + c * c);
        wr = sqrtf(x * x + y * y + z * z);
    };

    float r[9], p[3], e[6];
    fkdev::tcp_walk<float>(n, s_tab, s_itab, QReg<NMAX>{q}, r, p);
    pose_err(tr, tp, r, p, e);
    float wp, wr;
    weighted(e, wp, wr);
    float sc = sqrtf(wp * wp + wr * wr);

    int status = IKS_MAX_ITER;
    if (!isfinite(sc)) status = IKS_NAN;
    float lam = fmaxf(P.lam_min, P.lam0 * fminf(1.f, fmaxf(1e-3f, sc * 10.f)));
    float J[6][NMAX];
    bool jvalid = false;
    float A[6][6];
    float ew[6];
    float ref_sc = sc;
    int it = 0;

    for (; it < P.max_iter && status == IKS_MAX_ITER; ++it) {
        if (wp < P.tol_pos && wr < P.tol_rot) {
            status = IKS_CONVERGED;
            break;
        }
        if (!jvalid) {
            jac_walk<NMAX>(n, s_tab, s_itab, q, p, J);
#pragma unroll
            for (int a = 0; a < 6; ++a) {
#pragma unroll
                for (int d = 0; d < NMAX; ++d) J[a][d] *= P.w[a];
                ew[a] = P.w[a] * e[a];
            }
#pragma unroll
            for (int a = 0; a < 6; ++a)
#pragma unroll
                for (int b = 0; b <= a; ++b) {
                    float s = 0.f;
#pragma unroll
                    for (int d = 0; d < NMAX; ++d) s += J[a][d] * J[b][d];
                    A[a][b] = s;
                }
            jvalid = true;
        }
        float y[6];
        bool ok = false;
        for (int attempt = 0; attempt < 4 && !ok; ++attempt) {
            ok = chol_solve6(A, lam * lam, ew, y);
            if (!ok) lam *= 10.f;
        }
        if (!ok) {
            status = IKS_STALLED;
            break;
        }
        float dq[NMAX];
        float ratio = 0.f;
#pragma unroll
        for (int d = 0; d < NMAX; ++d) {
            float s = 0.f;
#pragma unroll
            for (int a = 0; a < 6; ++a) s += J[a][d] * y[a];
            dq[d] = s;
            if (d < nd) ratio = fmaxf(ratio, fabsf(s) / P.cap[d]);
        }
        const float scale = (ratio > 1.f) ? 1.f / ratio : 1.f;
        float qn[NMAX];
#pragma unroll
        for (int d = 0; d < NMAX; ++d)
            qn[d] = (d < nd) ? fminf(fmaxf(q[d] + dq[d] * scale, P.lo[d]), P.hi[d]) : 0.f;

        float rn[9], pn[3], en[6];
        fkdev::tcp_walk<float>(n, s_tab, s_itab, QReg<NMAX>{qn}, rn, pn);
        pose_err(tr, tp, rn, pn, en);
        float wpn, wrn;
        weighted(en, wpn, wrn);
        const float scn = sqrtf(wpn * wpn + wrn * wrn);
        if (!isfinite(scn)) {
            status = IKS_NAN;
            break;
        }
        if (scn > sc) {  // reject: bigger lambda, same J
            lam *= 10.f;
            if (lam >= 1e3f) {
                status = IKS_STALLED;
                break;
            }
        } else {  // accept
#pragma unroll
            for (int d = 0; d < NMAX; ++d) q[d] = qn[d];
#pragma unroll
            for (int k = 0; k < 9; ++k) r[k] = rn[k];
#pragma unroll
            for (int k = 0; k < 3; ++k) p[k] = pn[k];
#pragma unroll
            for (int k = 0; k < 6; ++k) e[k] = en[k];
            wp = wpn;
            wr = wrn;
            sc = scn;
            lam = fmaxf(0.5f * lam, P.lam_min);
            jvalid = false;
        }
        if ((it + 1) % 10 == 0) {  // stall: less than 10 % reduction over 10 iterations
            if (sc > 0.9f * ref_sc) {
                status = IKS_STALLED;
                break;
            }
            ref_sc = sc;
        }
    }
    if (status == IKS_MAX_ITER && wp < P.tol_pos && wr < P.tol_rot) status = IKS_CONVERGED;

    float ep = sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    float er = sqrtf(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]);
#pragma unroll
    for (int d = 0; d < NMAX; ++d)
        if (d < nd) qout[d * ld + i] = q[d];
    st[i] = status;
    perr[i] = ep;
    rerr[i] = er;
    iters[i] = it;
}

template <int NMAX>
void launch(int n, const void* d_tab, const int* d_itab, const IkParams& prm, const float* tgt,
            const float* q0, long long ld, float* qout, int* st, float* perr, float* rerr,
            int* iters, long long count, cudaStream_t stream) {
    const long long grid = (count + IK_THREADS - 1) / IK_THREADS;
    ik_kernel<NMAX><<<static_cast<unsigned>(grid), IK_THREADS, 0, stream>>>(
        n, static_cast<const float*>(d_tab), d_itab, prm, tgt, q0, ld, qout, st, perr, rerr, iters,
        count);
}

}  // namespace

void ik_launch(int n, const void* d_tab, const int* d_itab, const IkParams& prm,
               const float* d_tgt, const float* d_q0, long long ld, float* d_qout, int* d_st,
               float* d_perr, float* d_rerr, int* d_iters, long long count, cudaStream_t stream) {
    if (count <= 0) return;
    if (prm.nd <= 6)
        launch<6>(n, d_tab, d_itab, prm, d_tgt, d_q0, ld, d_qout, d_st, d_perr, d_rerr, d_iters,
                  count, stream);
    else
        launch<12>(n, d_tab, d_itab, prm, d_tgt, d_q0, ld, d_qout, d_st, d_perr, d_rerr, d_iters,
                   count, stream);
    CUDA_CHECK(cudaGetLastError());
#ifndef NDEBUG
    CUDA_CHECK(cudaStreamSynchronize(stream));
#endif
}

}  // namespace gpu
