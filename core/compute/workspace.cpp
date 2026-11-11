#include "compute/workspace.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "compute/parallel.hpp"
#include "compute/philox.hpp"
#include "kin/fk_batch.hpp"

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace compute {

double reach_bound(const kin::CompiledModel& cm, const std::vector<double>& lo,
                   const std::vector<double>& hi) {
    const std::vector<int> path = kin::active_path(cm);
    double R = cm.tool.translation().norm();
    for (int idx : path) {
        const kin::CompiledJoint& j = cm.joints[idx];
        R += j.pre.translation().norm() + j.post.translation().norm();
        if (j.type == urm::JointType::Prismatic) {
            const std::size_t s = static_cast<std::size_t>(j.src_q_index);
            const double a = j.mult * lo[s] + j.offset;
            const double b = j.mult * hi[s] + j.offset;
            R += std::max(std::abs(a), std::abs(b)) * j.axis.norm();
        }
    }
    return R;
}

std::size_t total_samples(const WorkspaceOptions& opt, int n_dof) {
    if (opt.mode != "grid") return opt.n_samples;
    unsigned long long prod = 1;
    for (int d = 0; d < n_dof; ++d) {
        const unsigned long long dv = static_cast<unsigned long long>(opt.divisions[d]);
        if (prod > (1ull << 62) / dv) return static_cast<std::size_t>(~0ull);  // overflow
        prod *= dv;
    }
    return static_cast<std::size_t>(prod);
}

WorkspaceGrid resolve_grid(const kin::CompiledModel& cm, const std::vector<double>& lo,
                           const std::vector<double>& hi, const WorkspaceOptions& opt) {
    const std::size_t nd = static_cast<std::size_t>(cm.n_dof);
    if (opt.mode != "random" && opt.mode != "grid")
        throw std::invalid_argument("sample_workspace: mode must be 'random' or 'grid'");
    if (lo.size() != nd || hi.size() != nd)
        throw std::invalid_argument("sample_workspace: lo/hi must have n_dof = " +
                                    std::to_string(nd) + " entries");
    for (std::size_t d = 0; d < nd; ++d) {
        if (!std::isfinite(lo[d]) || !std::isfinite(hi[d]) || lo[d] > hi[d])
            throw std::invalid_argument("sample_workspace: bad joint limits (need finite lo <= hi)");
    }
    if (!(opt.voxel_size > 0.0) || !std::isfinite(opt.voxel_size))
        throw std::invalid_argument("sample_workspace: voxel_size must be > 0");
    if (opt.n_samples == 0)
        throw std::invalid_argument("sample_workspace: n_samples must be > 0");
    if (opt.max_points == 0 && opt.want_points)
        throw std::invalid_argument("sample_workspace: max_points must be > 0");
    if (opt.chunk < 0 || (opt.chunk > 0 && opt.chunk % 256 != 0))
        throw std::invalid_argument("sample_workspace: chunk must be 0 or a positive multiple of 256");
    if (nd > 64) throw std::invalid_argument("sample_workspace: more than 64 DOF is not supported");
    if (opt.mode == "grid") {
        if (opt.divisions.size() != nd)
            throw std::invalid_argument("sample_workspace: divisions must have n_dof = " +
                                        std::to_string(nd) + " entries");
        for (int dv : opt.divisions)
            if (dv < 1) throw std::invalid_argument("sample_workspace: every division must be >= 1");
        const std::size_t tot = total_samples(opt, cm.n_dof);
        if (tot > opt.n_samples)
            throw std::invalid_argument("sample_workspace: grid has " + std::to_string(tot) +
                                        " samples (product of divisions), over the cap n_samples = " +
                                        std::to_string(opt.n_samples));
    }
    kin::active_path(cm);  // throws std::invalid_argument when tcp_link is unreachable

    double blo[3], bhi[3];
    if (opt.bounds_set) {
        for (int c = 0; c < 3; ++c) {
            blo[c] = opt.bounds_lo[c];
            bhi[c] = opt.bounds_hi[c];
            if (!std::isfinite(blo[c]) || !std::isfinite(bhi[c]) || !(bhi[c] > blo[c]))
                throw std::invalid_argument("sample_workspace: bounds need finite hi > lo");
        }
    } else {
        const double R = reach_bound(cm, lo, hi) + opt.voxel_size;
        for (int c = 0; c < 3; ++c) {
            blo[c] = -R;
            bhi[c] = R;
        }
    }
    WorkspaceGrid g;
    g.voxel = opt.voxel_size;
    long double nv = 1.0L;
    for (int c = 0; c < 3; ++c) {
        const double cells = std::ceil((bhi[c] - blo[c]) / opt.voxel_size);
        if (!(cells >= 1.0) || cells > 4.0e9)
            throw std::invalid_argument("sample_workspace: grid too large; maximum allowed is " +
                                        std::to_string(kMaxOccupiedVoxels) + " voxels");
        g.dims[c] = static_cast<int>(std::min(cells, 2147483647.0));
        g.origin[c] = blo[c];
        nv *= static_cast<long double>(cells);
    }
    if (nv > static_cast<long double>(kMaxOccupiedVoxels))
        throw std::invalid_argument("sample_workspace: grid has " +
                                    std::to_string(static_cast<double>(nv)) +
                                    " voxels; maximum allowed is " +
                                    std::to_string(kMaxOccupiedVoxels) +
                                    " (1024^3); use a larger voxel_size");
    g.n_voxels = static_cast<std::size_t>(nv);
    if (opt.want_counts && g.n_voxels * 4 > kMaxCountsBytes)
        throw std::invalid_argument("sample_workspace: counts grid has " +
                                    std::to_string(g.n_voxels) + " voxels; maximum allowed is " +
                                    std::to_string(kMaxCountsBytes / 4) +
                                    " (256 MB); use a larger voxel_size or want_counts=False");
    return g;
}

SamplerTables make_sampler(const std::vector<double>& lo, const std::vector<double>& hi,
                           const WorkspaceOptions& opt) {
    SamplerTables s;
    s.grid = (opt.mode == "grid");
    s.seed = opt.seed;
    s.n_dof = static_cast<int>(lo.size());
    s.lo.resize(lo.size());
    s.a.resize(lo.size());
    s.div.assign(lo.size(), 1);
    for (std::size_t d = 0; d < lo.size(); ++d) {
        const float l = static_cast<float>(lo[d]);
        const float h = static_cast<float>(hi[d]);
        if (!s.grid) {
            s.lo[d] = l;
            s.a[d] = h - l;
        } else {
            const int dv = opt.divisions[d];
            s.div[d] = dv;
            if (dv == 1) {  // single division: the center of the range
                s.lo[d] = l + 0.5f * (h - l);
                s.a[d] = 0.0f;
            } else {
                s.lo[d] = l;
                s.a[d] = (h - l) / static_cast<float>(dv - 1);  // endpoints included
            }
        }
    }
    return s;
}

void sample_q_host(const SamplerTables& s, std::size_t start, std::size_t n, float* out) {
    for (std::size_t i = 0; i < n; ++i) {
        float* q = out + i * static_cast<std::size_t>(s.n_dof);
        if (s.grid)
            fill_q_grid(q, s.n_dof, start + i, s.lo.data(), s.a.data(), s.div.data());
        else
            fill_q_random(q, s.n_dof, s.seed, start + i, s.lo.data(), s.a.data());
    }
}

void bitset_to_voxels(const std::uint32_t* words, const WorkspaceGrid& g,
                      std::vector<std::int32_t>& out) {
    out.clear();
    const std::size_t nw = (g.n_voxels + 31) / 32;
    const std::size_t ny = static_cast<std::size_t>(g.dims[1]);
    const std::size_t nz = static_cast<std::size_t>(g.dims[2]);
    for (std::size_t w = 0; w < nw; ++w) {
        std::uint32_t v = words[w];
        while (v) {
#if defined(_MSC_VER)
            unsigned long b;
            _BitScanForward(&b, v);
            const std::size_t bit = b;
#else
            const std::size_t bit = static_cast<std::size_t>(__builtin_ctz(v));
#endif
            v &= v - 1;
            const std::size_t lin = w * 32 + bit;
            out.push_back(static_cast<std::int32_t>(lin / (ny * nz)));
            out.push_back(static_cast<std::int32_t>((lin / nz) % ny));
            out.push_back(static_cast<std::int32_t>(lin % nz));
        }
    }
}

WorkspaceResult sample_workspace_cpu(const kin::CompiledModel& cm, const std::vector<double>& lo,
                                     const std::vector<double>& hi, const WorkspaceOptions& opt,
                                     const std::atomic<bool>* cancel, const ProgressFn& progress) {
    const auto t0 = std::chrono::steady_clock::now();
    WorkspaceResult res;
    res.grid = resolve_grid(cm, lo, hi, opt);
    const SamplerTables smp = make_sampler(lo, hi, opt);
    const std::size_t total = total_samples(opt, cm.n_dof);
    const std::size_t nd = static_cast<std::size_t>(cm.n_dof);
    res.n_samples = total;
    const std::size_t nw = (res.grid.n_voxels + 31) / 32;
    res.occupied.assign(nw, 0u);
    if (opt.want_counts) res.counts.assign(res.grid.n_voxels, 0u);
    const std::size_t n_pts = opt.want_points ? std::min(opt.max_points, total) : 0;
    res.points.assign(n_pts * 3, 0.0f);

    const WorkspaceGrid& g = res.grid;
    const double ox = g.origin[0], oy = g.origin[1], oz = g.origin[2], vox = g.voxel;
    const std::size_t nx = static_cast<std::size_t>(g.dims[0]);
    const std::size_t ny = static_cast<std::size_t>(g.dims[1]);
    const std::size_t nz = static_cast<std::size_t>(g.dims[2]);
    std::uint32_t* occ = res.occupied.data();
    std::uint32_t* cnt = opt.want_counts ? res.counts.data() : nullptr;
    float* pts = res.points.data();
    std::atomic<std::size_t> outside{0};

    const std::size_t chunk = opt.chunk > 0 ? static_cast<std::size_t>(opt.chunk) : (1u << 20);
    constexpr std::size_t kBlock = 4096;
    std::size_t done = 0;
    while (done < total) {
        if (cancel && cancel->load(std::memory_order_relaxed)) {
            res.cancelled = true;
            break;
        }
        const std::size_t cn = std::min(chunk, total - done);
        const std::size_t base = done;
        compute::parallel_for(cn, opt.n_threads, kBlock, [&](std::size_t b, std::size_t e) {
            std::vector<float> qf(kBlock * nd);
            std::vector<double> qd(kBlock * nd);
            std::size_t local_out = 0;
            for (std::size_t s0 = b; s0 < e; s0 += kBlock) {
                const std::size_t bn = std::min(kBlock, e - s0);
                sample_q_host(smp, base + s0, bn, qf.data());
                for (std::size_t i = 0; i < bn; ++i)  // SoA double, rounded float q
                    for (std::size_t d = 0; d < nd; ++d)
                        qd[d * bn + i] = static_cast<double>(qf[i * nd + d]);
                const kin::FkBatchResult r = kin::fk_batch_cpu(cm, qd.data(), bn, bn, 1);
                for (std::size_t i = 0; i < bn; ++i) {
                    const double px = r.position[i], py = r.position[bn + i],
                                 pz = r.position[2 * bn + i];
                    const std::size_t gi = base + s0 + i;
                    if (gi < n_pts) {
                        pts[gi * 3] = static_cast<float>(px);
                        pts[gi * 3 + 1] = static_cast<float>(py);
                        pts[gi * 3 + 2] = static_cast<float>(pz);
                    }
                    const double fx = std::floor((px - ox) / vox);
                    const double fy = std::floor((py - oy) / vox);
                    const double fz = std::floor((pz - oz) / vox);
                    if (!(fx >= 0.0 && fx < static_cast<double>(nx) && fy >= 0.0 &&
                          fy < static_cast<double>(ny) && fz >= 0.0 &&
                          fz < static_cast<double>(nz))) {
                        ++local_out;
                        continue;
                    }
                    const std::size_t lin =
                        (static_cast<std::size_t>(fx) * ny + static_cast<std::size_t>(fy)) * nz +
                        static_cast<std::size_t>(fz);
                    std::atomic_ref<std::uint32_t>(occ[lin >> 5])
                        .fetch_or(1u << (lin & 31), std::memory_order_relaxed);
                    if (cnt)
                        std::atomic_ref<std::uint32_t>(cnt[lin]).fetch_add(
                            1u, std::memory_order_relaxed);
                }
            }
            outside.fetch_add(local_out, std::memory_order_relaxed);
        });
        done += cn;
        ++res.chunks;
        if (progress) progress(done, total);
    }
    res.n_done = done;
    res.n_outside = outside.load();
    res.chunk = static_cast<long long>(chunk);
    if (res.cancelled && n_pts > done) res.points.resize(done * 3);
    bitset_to_voxels(res.occupied.data(), res.grid, res.voxels);
    res.backend = "cpu";
    res.total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return res;
}

}  // namespace compute
