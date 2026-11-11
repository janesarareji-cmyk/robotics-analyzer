/**
 * workspace_gpu.cpp
 * R5.3: GPU workspace sampling driver.
 *
 * Persistent device data: occupied bitset, optional counts, joint tables, sampler
 * constants. Per chunk the kernel generates q on the device, so no q buffer exists.
 * Only want_points needs per-chunk buffers (2 device + 2 pinned, one per stream).
 * Chunk size: compute::chunk_size(free VRAM, budget, persistent, bytes/sample), then
 * the first chunk is timed and later chunks are scaled to about 50 ms per launch.
 * Errors: out of memory -> halve the chunk (below 65536 -> CPU); sticky CUDA error ->
 * GPU dead for the process -> CPU. Explicit budget too small for the grid ->
 * std::invalid_argument (ValueError); automatic budget too small -> CPU fallback.
 */
#include "workspace_gpu.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <map>
#include <vector>

#include "compute/budget.hpp"
#include "gpu_manager.cuh"
#include "workspace_kernel.cuh"

namespace gpu {

namespace {

constexpr double kTargetKernelMs = 50.0;  // TDR safety: R5 Decisions 9

struct Oom {
    std::string what;
};

struct DevMem {
    std::map<void*, size_t> m;
    size_t cur = 0, peak = 0;
    cudaError_t alloc(void** p, size_t bytes) {
        bytes = std::max<size_t>(bytes, 16);
        cudaError_t e = gpu::checked_malloc(p, bytes);
        if (e != cudaSuccess) {
            cudaGetLastError();
            *p = nullptr;
            return e;
        }
        m[*p] = bytes;
        cur += bytes;
        peak = std::max(peak, cur);
        return cudaSuccess;
    }
    /// Allocates or throws Oom (out of memory) / CudaError (anything else).
    void must(void** p, size_t bytes, const char* what) {
        const cudaError_t e = alloc(p, bytes);
        if (e == cudaSuccess) return;
        if (gpu::is_oom(e)) throw Oom{what};
        gpu::cuda_check(e, what, __FILE__, __LINE__);
    }
    void release(void*& p) {
        if (!p) return;
        auto it = m.find(p);
        cudaFree(p);
        if (it != m.end()) {
            cur -= it->second;
            m.erase(it);
        }
        p = nullptr;
    }
    ~DevMem() {
        for (auto& kv : m) cudaFree(kv.first);
    }
};

struct Slot {
    cudaStream_t st = nullptr;
    cudaEvent_t e0 = nullptr, e1 = nullptr;
    float* d_pts = nullptr;
    float* h_pts = nullptr;
    size_t start = 0, cn = 0, pt_n = 0;
    bool valid = false;
};

struct Slots {
    Slot s[2];
    ~Slots() {
        for (Slot& x : s) {
            if (x.h_pts) cudaFreeHost(x.h_pts);
            if (x.e0) cudaEventDestroy(x.e0);
            if (x.e1) cudaEventDestroy(x.e1);
            if (x.st) cudaStreamDestroy(x.st);
        }
    }
};

}  // namespace

bool workspace_gpu(const FkPathTables& tables, const compute::SamplerTables& smp,
                   const compute::WorkspaceGrid& grid, const compute::WorkspaceOptions& opt,
                   std::size_t total, const std::atomic<bool>* cancel,
                   const compute::ProgressFn& progress, compute::WorkspaceResult& out,
                   std::string& reason) {
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    if (gpu::is_dead()) {
        reason = "GPU marked dead by an earlier error";
        return false;
    }
    int dev_count = 0;
    if (cudaGetDeviceCount(&dev_count) != cudaSuccess || dev_count <= 0) {
        cudaGetLastError();
        reason = "no CUDA device";
        return false;
    }
    gpu::MemInfo mi = gpu::mem_info();
    if (!mi.ok) {
        reason = "cudaMemGetInfo failed: " + mi.error;
        return false;
    }

    const size_t nvox = grid.n_voxels;
    const size_t occ_words = (nvox + 31) / 32;
    const size_t occ_bytes = occ_words * 4;
    const size_t cnt_bytes = opt.want_counts ? nvox * 4 : 0;
    std::vector<char> tab;
    std::vector<int> itab;
    fk_pack_tables(tables, false, tab, itab);
    const size_t nd = static_cast<size_t>(smp.n_dof);
    const size_t persistent = occ_bytes + cnt_bytes + tab.size() + itab.size() * sizeof(int) +
                              nd * (2 * sizeof(float) + sizeof(int)) + sizeof(unsigned long long) +
                              4 * 16;  // allocation granularity slack
    const bool explicit_budget = opt.budget_bytes > 0;
    const size_t budget = explicit_budget ? opt.budget_bytes : mi.total_bytes;
    const size_t n_pts = opt.want_points ? std::min(opt.max_points, total) : 0;
    const long long bps = n_pts ? 12 : 4;  // nominal device bytes per sample
    const bool forced = opt.chunk > 0;
    const long long min_cap = forced ? 256 : 65536;

    if (explicit_budget && persistent + static_cast<size_t>(2 * bps * min_cap) > budget) {
        throw std::invalid_argument(
            "sample_workspace: budget_bytes = " + std::to_string(budget) +
            " is too small for this grid; at least " +
            std::to_string(persistent + static_cast<size_t>(2 * bps * min_cap)) +
            " bytes are needed (use a larger voxel_size or budget)");
    }
    long long cap;
    if (forced) {
        cap = opt.chunk;
    } else {
        cap = compute::chunk_size(static_cast<long long>(mi.free_bytes),
                                  static_cast<long long>(budget),
                                  static_cast<long long>(persistent), bps);
        if (cap == 0) {
            reason = "not enough free VRAM for the grid";
            return false;
        }
    }
    cap = std::min(cap, static_cast<long long>((total + 255) / 256 * 256));
    cap = std::max<long long>(cap, 256);

    compute::WorkspaceResult res;
    res.grid = grid;
    res.n_samples = total;
    res.budget_bytes = budget;
    DevMem dev;
    Slots sl;
    try {
        int sm_count = 0;
        CUDA_CHECK(cudaDeviceGetAttribute(&sm_count, cudaDevAttrMultiProcessorCount, 0));

        WsKernelArgs a;
        a.n = tables.n;
        a.n_dof = smp.n_dof;
        a.grid_mode = smp.grid;
        a.seed = smp.seed;
        for (int c = 0; c < 3; ++c) {
            a.origin[c] = static_cast<float>(grid.origin[c]);
            a.dims[c] = grid.dims[c];
        }
        a.voxel = static_cast<float>(grid.voxel);

        void *d_tab = nullptr, *d_itab = nullptr, *d_lo = nullptr, *d_a = nullptr, *d_div = nullptr,
             *d_occ = nullptr, *d_cnt = nullptr, *d_out = nullptr;
        try {
            dev.must(&d_tab, tab.size(), "tab");
            dev.must(&d_itab, itab.size() * sizeof(int), "itab");
            dev.must(&d_lo, nd * sizeof(float), "lo");
            dev.must(&d_a, nd * sizeof(float), "a");
            dev.must(&d_div, nd * sizeof(int), "div");
            dev.must(&d_occ, occ_bytes, "occupied");
            if (opt.want_counts) dev.must(&d_cnt, cnt_bytes, "counts");
            dev.must(&d_out, sizeof(unsigned long long), "outside");
        } catch (const Oom& o) {
            reason = std::string("out of GPU memory (") + o.what + ")";
            return false;
        }
        CUDA_CHECK(cudaMemcpy(d_tab, tab.data(), tab.size(), cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(d_itab, itab.data(), itab.size() * sizeof(int), cudaMemcpyHostToDevice));
        if (nd) {
            CUDA_CHECK(cudaMemcpy(d_lo, smp.lo.data(), nd * sizeof(float), cudaMemcpyHostToDevice));
            CUDA_CHECK(cudaMemcpy(d_a, smp.a.data(), nd * sizeof(float), cudaMemcpyHostToDevice));
            CUDA_CHECK(cudaMemcpy(d_div, smp.div.data(), nd * sizeof(int), cudaMemcpyHostToDevice));
        }
        CUDA_CHECK(cudaMemset(d_occ, 0, occ_bytes));
        if (d_cnt) CUDA_CHECK(cudaMemset(d_cnt, 0, cnt_bytes));
        CUDA_CHECK(cudaMemset(d_out, 0, sizeof(unsigned long long)));
        a.d_tab = static_cast<const float*>(d_tab);
        a.d_itab = static_cast<const int*>(d_itab);
        a.d_lo = static_cast<const float*>(d_lo);
        a.d_a = static_cast<const float*>(d_a);
        a.d_div = static_cast<const int*>(d_div);
        a.d_occ = static_cast<unsigned int*>(d_occ);
        a.d_counts = static_cast<unsigned int*>(d_cnt);
        a.d_outside = static_cast<unsigned long long*>(d_out);

        for (Slot& s : sl.s) {
            CUDA_CHECK(cudaStreamCreateWithFlags(&s.st, cudaStreamNonBlocking));
            CUDA_CHECK(cudaEventCreate(&s.e0));
            CUDA_CHECK(cudaEventCreate(&s.e1));
        }
        // Per-chunk buffers exist only for the optional point output. OOM -> halve the chunk.
        if (n_pts) {
            for (;;) {
                const size_t pb = std::min<size_t>(static_cast<size_t>(cap), n_pts) * 12;
                bool ok = true;
                for (Slot& s : sl.s) {
                    void* dp = s.d_pts;
                    if (dev.alloc(&dp, pb) != cudaSuccess) {
                        ok = false;
                        break;
                    }
                    s.d_pts = static_cast<float*>(dp);
                    if (cudaMallocHost(reinterpret_cast<void**>(&s.h_pts), pb) != cudaSuccess) {
                        cudaGetLastError();
                        s.h_pts = nullptr;
                        ok = false;
                        break;
                    }
                }
                if (ok) break;
                for (Slot& s : sl.s) {
                    void* dp = s.d_pts;
                    dev.release(dp);
                    s.d_pts = nullptr;
                    if (s.h_pts) cudaFreeHost(s.h_pts);
                    s.h_pts = nullptr;
                }
                cap = (cap / 2) / 256 * 256;
                if (cap < min_cap || cap < 256) {
                    reason = "out of GPU memory (point buffers)";
                    return false;
                }
            }
        }
        res.chunk = cap;

        long long cur = forced ? cap : std::min<long long>(cap, (1LL << 20));
        cur = std::max<long long>(cur - cur % 256, 256);
        size_t pos = 0;
        int s = 0;

        auto finish = [&](int si) {
            Slot& x = sl.s[si];
            CUDA_CHECK(cudaStreamSynchronize(x.st));
            float ms = 0.f;
            CUDA_CHECK(cudaEventElapsedTime(&ms, x.e0, x.e1));
            res.kernel_ms += ms;
            if (x.pt_n)
                std::memcpy(res.points.data() + x.start * 3, x.h_pts, x.pt_n * 12);
            x.valid = false;
            if (!forced && ms > 0.f) {  // keep each launch near the 50 ms target
                long long nc = static_cast<long long>(static_cast<double>(x.cn) *
                                                      (kTargetKernelMs / ms));
                nc = std::min(nc, cap);
                nc = std::max(nc, std::min(cap, 65536LL));
                nc -= nc % 256;
                cur = std::max(nc, 256LL);
            }
        };

        res.points.assign(n_pts * 3, 0.0f);
        while (pos < total) {
            if (cancel && cancel->load(std::memory_order_relaxed)) {
                res.cancelled = true;
                break;
            }
            Slot& x = sl.s[s];
            if (x.valid) finish(s);
            const size_t cn = std::min<size_t>(static_cast<size_t>(cur), total - pos);
            a.start = static_cast<long long>(pos);
            a.count = static_cast<long long>(cn);
            const size_t pt_n = pos < n_pts ? std::min(cn, n_pts - pos) : 0;
            a.d_points = pt_n ? x.d_pts : nullptr;
            a.pt_count = static_cast<long long>(pt_n);
            CUDA_CHECK(cudaEventRecord(x.e0, x.st));
            const int blocks = static_cast<int>(std::min<long long>(
                (static_cast<long long>(cn) + FK_THREADS - 1) / FK_THREADS, 8LL * sm_count));
            ws_launch(a, blocks, x.st);
            CUDA_CHECK(cudaEventRecord(x.e1, x.st));
            if (pt_n)
                CUDA_CHECK(cudaMemcpyAsync(x.h_pts, x.d_pts, pt_n * 12, cudaMemcpyDeviceToHost, x.st));
            x.start = pos;
            x.cn = cn;
            x.pt_n = pt_n;
            x.valid = true;
            pos += cn;
            ++res.chunks;
            if (res.chunks == 1) finish(s);  // time the first chunk before sizing the rest
            if (progress) progress(pos, total);
            s ^= 1;
        }
        for (int k = 0; k < 2; ++k) {
            const int si = s ^ k;  // older chunk first
            if (sl.s[si].valid) finish(si);
        }
        res.n_done = pos;
        if (res.cancelled && n_pts > pos) res.points.resize(pos * 3);

        unsigned long long outside = 0;
        CUDA_CHECK(cudaMemcpy(&outside, d_out, sizeof outside, cudaMemcpyDeviceToHost));
        res.n_outside = static_cast<size_t>(outside);
        res.occupied.resize(occ_words);
        CUDA_CHECK(cudaMemcpy(res.occupied.data(), d_occ, occ_bytes, cudaMemcpyDeviceToHost));
        if (opt.want_counts) {
            res.counts.resize(nvox);
            CUDA_CHECK(cudaMemcpy(res.counts.data(), d_cnt, cnt_bytes, cudaMemcpyDeviceToHost));
        }

        // Free the point buffers, then compact the occupied voxels on the device if memory allows.
        for (Slot& x : sl.s) {
            void* dp = x.d_pts;
            dev.release(dp);
            x.d_pts = nullptr;
            if (x.h_pts) cudaFreeHost(x.h_pts);
            x.h_pts = nullptr;
        }
        bool done_gpu = false;
        const unsigned long long m_set = ws_count_bits(a.d_occ, occ_words, sl.s[0].st);
        if (m_set > 0 && nvox <= 0x7fffffffu) {
            const size_t temp = ws_compact(nullptr, 0, a.d_occ, static_cast<unsigned int>(nvox),
                                           nullptr, nullptr, sl.s[0].st);
            const size_t need = m_set * 4 + temp + 16;
            if (!explicit_budget || dev.cur + need <= budget) {
                void *d_idx = nullptr, *d_tmp = nullptr, *d_num = nullptr;
                bool ok = dev.alloc(&d_idx, m_set * 4) == cudaSuccess;
                ok = ok && dev.alloc(&d_tmp, temp) == cudaSuccess;
                ok = ok && dev.alloc(&d_num, sizeof(int)) == cudaSuccess;
                if (ok) {
                    (void)ws_compact(d_tmp, temp, a.d_occ, static_cast<unsigned int>(nvox),
                               static_cast<unsigned int*>(d_idx), static_cast<int*>(d_num),
                               sl.s[0].st);
                    int n_sel = 0;
                    std::vector<unsigned int> lin(m_set);
                    CUDA_CHECK(cudaMemcpyAsync(&n_sel, d_num, sizeof(int), cudaMemcpyDeviceToHost,
                                               sl.s[0].st));
                    CUDA_CHECK(cudaMemcpyAsync(lin.data(), d_idx, m_set * 4, cudaMemcpyDeviceToHost,
                                               sl.s[0].st));
                    CUDA_CHECK(cudaStreamSynchronize(sl.s[0].st));
                    if (static_cast<unsigned long long>(n_sel) == m_set) {
                        const size_t ny = static_cast<size_t>(grid.dims[1]);
                        const size_t nz = static_cast<size_t>(grid.dims[2]);
                        res.voxels.resize(m_set * 3);
                        for (size_t k = 0; k < m_set; ++k) {
                            const size_t l = lin[k];
                            res.voxels[k * 3] = static_cast<int>(l / (ny * nz));
                            res.voxels[k * 3 + 1] = static_cast<int>((l / nz) % ny);
                            res.voxels[k * 3 + 2] = static_cast<int>(l % nz);
                        }
                        done_gpu = true;
                    }
                }
                void* p1 = d_idx;
                void* p2 = d_tmp;
                void* p3 = d_num;
                dev.release(p1);
                dev.release(p2);
                dev.release(p3);
            }
        }
        res.gpu_compaction = done_gpu;
        if (!done_gpu) compute::bitset_to_voxels(res.occupied.data(), grid, res.voxels);
        CUDA_CHECK(cudaDeviceSynchronize());
    } catch (const gpu::CudaError& e) {
        cudaDeviceSynchronize();
        cudaGetLastError();
        if (gpu::is_dead()) {
            reason = std::string("sticky CUDA error: ") + e.what();
            return false;
        }
        throw;
    } catch (...) {
        cudaDeviceSynchronize();
        cudaGetLastError();
        throw;
    }
    res.vram_peak_bytes = dev.peak;
    res.backend = "cuda";
    res.total_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    out = std::move(res);
    return true;
}

void sample_q_gpu(const compute::SamplerTables& smp, std::size_t start, std::size_t n, float* out) {
    if (n == 0) return;
    DevMem dev;
    const size_t nd = static_cast<size_t>(smp.n_dof);
    void *d_lo = nullptr, *d_a = nullptr, *d_div = nullptr, *d_q = nullptr;
    dev.must(&d_lo, nd * sizeof(float), "lo");
    dev.must(&d_a, nd * sizeof(float), "a");
    dev.must(&d_div, nd * sizeof(int), "div");
    dev.must(&d_q, n * nd * sizeof(float), "q");
    if (nd) {
        CUDA_CHECK(cudaMemcpy(d_lo, smp.lo.data(), nd * sizeof(float), cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(d_a, smp.a.data(), nd * sizeof(float), cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(d_div, smp.div.data(), nd * sizeof(int), cudaMemcpyHostToDevice));
    }
    const int blocks = static_cast<int>(std::min<size_t>((n + FK_THREADS - 1) / FK_THREADS, 65535));
    ws_gen_q(smp.n_dof, static_cast<const float*>(d_lo), static_cast<const float*>(d_a),
             static_cast<const int*>(d_div), smp.grid, smp.seed, static_cast<long long>(start),
             static_cast<long long>(n), static_cast<float*>(d_q), blocks, nullptr);
    CUDA_CHECK(cudaDeviceSynchronize());
    CUDA_CHECK(cudaMemcpy(out, d_q, n * nd * sizeof(float), cudaMemcpyDeviceToHost));
}

}  // namespace gpu
