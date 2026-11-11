/**
 * ik_gpu.cpp
 * R5.4 part B: GPU IK driver. Per chunk: world target -> base frame (host, double) ->
 * float SoA, H2D, kernel, D2H, widen. Chunk size from the R5.1 budget, then scaled so
 * one launch takes about 50 ms (R5 Decisions 9). OOM -> halve, below the minimum -> the
 * caller uses the CPU.
 */
#include "ik_gpu.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#include "compute/budget.hpp"
#include "compute/parallel.hpp"
#include "gpu_manager.cuh"

namespace gpu {

namespace {

constexpr double kTargetKernelMs = 50.0;
constexpr long long kMaxChunk = 1LL << 20;
constexpr long long kFirstChunk = 32768;

struct DevBuf {
    void* p = nullptr;
    ~DevBuf() {
        if (p) cudaFree(p);
    }
};

}  // namespace

bool ik_gpu_run(const IkGpuIn& in, IkGpuOut& out, std::string& reason) {
    const std::size_t N = in.N;
    const int nd = in.prm.nd;
    const std::size_t ndz = static_cast<std::size_t>(nd);
    out.q.assign(N * ndz, 0.0);
    out.status.assign(N, IKS_NAN);
    out.iters.assign(N, 0);
    out.perr.assign(N, 0.f);
    out.rerr.assign(N, 0.f);
    if (N == 0) return true;

    if (is_dead()) {
        reason = "GPU marked dead by an earlier error";
        return false;
    }
    int dev_count = 0;
    if (cudaGetDeviceCount(&dev_count) != cudaSuccess || dev_count <= 0) {
        cudaGetLastError();
        reason = "no CUDA device";
        return false;
    }

    const bool forced = in.forced_chunk > 0;
    const long long min_cap = forced ? 128 : 65536;
    const std::size_t floats_in = 12 + ndz, floats_out = ndz + 4;
    const long long bps = static_cast<long long>((floats_in + floats_out) * 4);
    const long long n_round = static_cast<long long>((N + 127) / 128 * 128);

    long long cap;
    if (forced) {
        cap = in.forced_chunk;
    } else {
        MemInfo mi = mem_info();
        if (!mi.ok) {
            reason = "cudaMemGetInfo failed: " + mi.error;
            return false;
        }
        const long long cs = compute::chunk_size(static_cast<long long>(mi.free_bytes),
                                                 static_cast<long long>(mi.total_bytes), 16384, bps);
        if (cs == 0) {
            reason = "not enough free VRAM";
            return false;
        }
        cap = std::min(cs, kMaxChunk);
    }
    cap = std::min(cap, n_round);

    std::vector<char> tab;
    std::vector<int> itab;
    fk_pack_tables(*in.tables, false, tab, itab);

    DevBuf d_tab, d_itab, d_data;
    try {
        for (;;) {
            cudaError_t e = checked_malloc(&d_data.p, static_cast<size_t>(cap) * bps);
            if (e == cudaSuccess) break;
            cudaGetLastError();
            d_data.p = nullptr;
            if (!is_oom(e)) cuda_check(e, "ik_gpu alloc", __FILE__, __LINE__);
            cap = (cap / 2) / 128 * 128;
            if (cap < min_cap || cap < 128) {
                reason = "out of GPU memory";
                return false;
            }
        }
        out.chunk = cap;
        CUDA_CHECK(cudaMalloc(&d_tab.p, std::max<size_t>(tab.size(), 16)));
        CUDA_CHECK(cudaMalloc(&d_itab.p, std::max<size_t>(itab.size() * sizeof(int), 16)));
        CUDA_CHECK(cudaMemcpy(d_tab.p, tab.data(), tab.size(), cudaMemcpyHostToDevice));
        if (!itab.empty())
            CUDA_CHECK(cudaMemcpy(d_itab.p, itab.data(), itab.size() * sizeof(int),
                                  cudaMemcpyHostToDevice));

        cudaEvent_t e0 = nullptr, e1 = nullptr;
        CUDA_CHECK(cudaEventCreate(&e0));
        CUDA_CHECK(cudaEventCreate(&e1));
        struct EvGuard {
            cudaEvent_t a, b;
            ~EvGuard() {
                cudaEventDestroy(a);
                cudaEventDestroy(b);
            }
        } evg{e0, e1};

        // Host staging (same layout as the device buffer, offsets from cn).
        std::vector<float> h_in(static_cast<std::size_t>(cap) * floats_in);
        std::vector<float> h_out(static_cast<std::size_t>(cap) * floats_out);

        const double* Mb = in.base;
        long long cur = forced ? cap : std::min(cap, kFirstChunk);
        std::size_t pos = 0;
        while (pos < N) {
            const std::size_t cn = std::min<std::size_t>(static_cast<std::size_t>(cur), N - pos);
            float* tg = h_in.data();            // [12][cn]
            float* qq = h_in.data() + 12 * cn;  // [nd][cn]
            compute::parallel_for(cn, in.n_threads, 8192, [&](std::size_t b, std::size_t e) {
                for (std::size_t i = b; i < e; ++i) {
                    const double* T = in.targets + (pos + i) * 16;
                    // base^-1 * T : R_b^T (p - p_b), R_b^T R
                    double v[12];
                    for (int a = 0; a < 3; ++a) {
                        double px = 0;
                        for (int k = 0; k < 3; ++k) px += Mb[k * 4 + a] * (T[k * 4 + 3] - Mb[k * 4 + 3]);
                        v[a] = px;
                        for (int c = 0; c < 3; ++c) {
                            double s = 0;
                            for (int k = 0; k < 3; ++k) s += Mb[k * 4 + a] * T[k * 4 + c];
                            v[3 + a * 3 + c] = s;
                        }
                    }
                    for (int k = 0; k < 12; ++k) tg[k * cn + i] = static_cast<float>(v[k]);
                    const double* q0i = in.q0_broadcast ? in.q0 : in.q0 + (pos + i) * ndz;
                    for (int d = 0; d < nd; ++d) qq[d * cn + i] = static_cast<float>(q0i[d]);
                }
            });

            float* d_f = static_cast<float*>(d_data.p);
            const float* d_tgt = d_f;
            const float* d_q0 = d_f + 12 * cn;
            float* d_qout = d_f + floats_in * cn;
            float* d_perr = d_qout + ndz * cn;
            float* d_rerr = d_perr + cn;
            int* d_st = reinterpret_cast<int*>(d_rerr + cn);
            int* d_it = d_st + cn;

            CUDA_CHECK(cudaMemcpy(d_data.p, h_in.data(), floats_in * cn * sizeof(float),
                                  cudaMemcpyHostToDevice));
            CUDA_CHECK(cudaEventRecord(e0, nullptr));
            ik_launch(in.tables->n, d_tab.p, static_cast<const int*>(d_itab.p), in.prm, d_tgt, d_q0,
                      static_cast<long long>(cn), d_qout, d_st, d_perr, d_rerr, d_it,
                      static_cast<long long>(cn), nullptr);
            CUDA_CHECK(cudaEventRecord(e1, nullptr));
            CUDA_CHECK(cudaMemcpy(h_out.data(), d_qout, floats_out * cn * sizeof(float),
                                  cudaMemcpyDeviceToHost));
            float ms = 0.f;
            CUDA_CHECK(cudaEventElapsedTime(&ms, e0, e1));
            out.kernel_ms += ms;
            ++out.chunks;

            const float* ho_q = h_out.data();
            const float* ho_pe = ho_q + ndz * cn;
            const float* ho_re = ho_pe + cn;
            const int* ho_st = reinterpret_cast<const int*>(ho_re + cn);
            const int* ho_it = ho_st + cn;
            for (std::size_t i = 0; i < cn; ++i) {
                for (int d = 0; d < nd; ++d)
                    out.q[(pos + i) * ndz + d] = static_cast<double>(ho_q[d * cn + i]);
                out.status[pos + i] = ho_st[i];
                out.iters[pos + i] = ho_it[i];
                out.perr[pos + i] = ho_pe[i];
                out.rerr[pos + i] = ho_re[i];
            }
            pos += cn;
            if (!forced && ms > 0.f) {  // keep each launch near the 50 ms target
                long long nc = static_cast<long long>(static_cast<double>(cn) * (kTargetKernelMs / ms));
                nc = std::min(nc, cap);
                nc = std::max(nc, std::min(cap, 4096LL));
                nc -= nc % 128;
                cur = std::max(nc, 128LL);
            }
        }
    } catch (...) {
        cudaDeviceSynchronize();
        cudaGetLastError();
        throw;
    }
    return true;
}

}  // namespace gpu
