/**
 * gpu_bridge.cpp
 * R5.1: optional pybind11 module for GPU compute. core_bridge never links CUDA.
 * Imports fine without a GPU (device_info reports available = False).
 */
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include "compute/budget.hpp"
#include "compute/parallel.hpp"
#include "fk_kernel.cuh"
#include "gpu_manager.cuh"
#include "kin/compiled_arrays.hpp"
#include "kin/fk_batch.hpp"
#include "compute/workspace.hpp"
#include "compute/workspace_py.hpp"
#include "workspace_gpu.hpp"
#include "ik_gpu.hpp"
#include "kin/ik.hpp"
#include "kin/ik_batch.hpp"
#include "kin/ik_py.hpp"

namespace py = pybind11;

namespace {

// ---------------------------------------------------------------------------
// R5.2 batch FK: pooled pinned/device buffers, 2 streams, chunked pipeline.
// ---------------------------------------------------------------------------

constexpr long long kMaxLaunchSamples = 1LL << 20;  // pool cap per slot (pinned + device)
constexpr double kTargetKernelMs = 50.0;            // TDR safety: R5 Decisions 9
constexpr size_t kTabCap = 16384, kItabCap = 1024;  // fits 64 joints in fp64

struct Slot {
    cudaStream_t st = nullptr;
    cudaEvent_t e0 = nullptr, e1 = nullptr;
    void *h_in = nullptr, *h_out = nullptr, *d_in = nullptr, *d_out = nullptr;
    size_t in_bytes = 0, out_bytes = 0;
};

struct Pool {
    Slot slot[2];
    void* d_tab = nullptr;
    int* d_itab = nullptr;
    bool init = false;
};

Pool g_pool;
std::mutex g_fk_mutex;

void pool_free_buffers() {
    for (Slot& s : g_pool.slot) {
        if (s.h_in) cudaFreeHost(s.h_in);
        if (s.h_out) cudaFreeHost(s.h_out);
        if (s.d_in) cudaFree(s.d_in);
        if (s.d_out) cudaFree(s.d_out);
        s.h_in = s.h_out = s.d_in = s.d_out = nullptr;
        s.in_bytes = s.out_bytes = 0;
    }
}

/// Returns cudaSuccess or an OOM code (buffers freed); throws on other errors.
cudaError_t ensure_pool(size_t in_b, size_t out_b) {
    in_b = std::max<size_t>(in_b, 16);
    out_b = std::max<size_t>(out_b, 16);
    if (!g_pool.init) {
        for (Slot& s : g_pool.slot) {
            CUDA_CHECK(cudaStreamCreateWithFlags(&s.st, cudaStreamNonBlocking));
            CUDA_CHECK(cudaEventCreate(&s.e0));
            CUDA_CHECK(cudaEventCreate(&s.e1));
        }
        CUDA_CHECK(cudaMalloc(&g_pool.d_tab, kTabCap));
        CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&g_pool.d_itab), kItabCap));
        g_pool.init = true;
    }
    if (g_pool.slot[0].in_bytes >= in_b && g_pool.slot[0].out_bytes >= out_b)
        return gpu::check_alloc();  // fake-OOM hook is consumed even when nothing is allocated
    pool_free_buffers();
    for (Slot& s : g_pool.slot) {
        cudaError_t e = gpu::checked_malloc(&s.d_in, in_b);
        if (e == cudaSuccess) e = gpu::checked_malloc(&s.d_out, out_b);
        if (e == cudaSuccess) e = cudaMallocHost(&s.h_in, in_b);
        if (e == cudaSuccess) e = cudaMallocHost(&s.h_out, out_b);
        if (e != cudaSuccess) {
            cudaGetLastError();
            pool_free_buffers();
            if (gpu::is_oom(e)) return e;
            gpu::cuda_check(e, "ensure_pool", __FILE__, __LINE__);
        }
        s.in_bytes = in_b;
        s.out_bytes = out_b;
    }
    return cudaSuccess;
}

struct RunStats {
    int chunks = 0;
    double kernel_ms = 0.0;
    long long chunk = 0;
    std::string reason;  // non-empty: GPU not used
};

struct Pending {
    size_t start = 0, cn = 0;
    bool valid = false;
};

template <typename T>
void pack_q(T* dst, const double* q, size_t start, size_t cn, size_t dof, int nth) {
    compute::parallel_for(cn, nth, 16384, [&](size_t b, size_t e) {
        for (size_t i = b; i < e; ++i) {
            const double* row = q + (start + i) * dof;
            for (size_t d = 0; d < dof; ++d) dst[d * cn + i] = static_cast<T>(row[d]);
        }
    });
}

/// Device output [12][cn] (T) -> double position (N,3) / rotation (N,3,3); base applied if world.
template <typename T>
void unpack_out(const T* src, size_t start, size_t cn, double* pos, double* rot, bool world,
                const double* Mb, int nth) {
    compute::parallel_for(cn, nth, 16384, [&](size_t b, size_t e) {
        for (size_t i = b; i < e; ++i) {
            double v[12];
            for (int k = 0; k < 12; ++k) v[k] = static_cast<double>(src[k * cn + i]);
            double* po = pos + (start + i) * 3;
            double* ro = rot + (start + i) * 9;
            if (world) {
                for (int a = 0; a < 3; ++a) {
                    po[a] = Mb[a * 4 + 0] * v[0] + Mb[a * 4 + 1] * v[1] + Mb[a * 4 + 2] * v[2] +
                            Mb[a * 4 + 3];
                    for (int c = 0; c < 3; ++c)
                        ro[a * 3 + c] = Mb[a * 4 + 0] * v[3 + 0 * 3 + c] +
                                        Mb[a * 4 + 1] * v[3 + 1 * 3 + c] +
                                        Mb[a * 4 + 2] * v[3 + 2 * 3 + c];
                }
            } else {
                for (int a = 0; a < 3; ++a) po[a] = v[a];
                for (int k = 0; k < 9; ++k) ro[k] = v[3 + k];
            }
        }
    });
}

/// Returns true if the GPU did the work. false: stats.reason says why (CPU fallback).
bool run_gpu(const gpu::FkPathTables& tables, bool fp64, const double* q, size_t N, int dof_i,
             double* pos, double* rot, bool world, const double* Mb, long long forced_chunk,
             int nth, RunStats& st) {
    if (gpu::is_dead()) {
        st.reason = "GPU marked dead by an earlier error";
        return false;
    }
    int dev_count = 0;
    if (cudaGetDeviceCount(&dev_count) != cudaSuccess || dev_count <= 0) {
        cudaGetLastError();
        st.reason = "no CUDA device";
        return false;
    }
    static int sm_count = 0;
    if (sm_count <= 0) {
        int v = 0;
        CUDA_CHECK(cudaDeviceGetAttribute(&v, cudaDevAttrMultiProcessorCount, 0));
        sm_count = v;
    }
    const size_t sT = fp64 ? sizeof(double) : sizeof(float);
    const size_t dof = static_cast<size_t>(dof_i);
    const bool forced = forced_chunk > 0;
    const long long min_cap = forced ? 256 : 65536;
    const long long n_round = static_cast<long long>((N + 255) / 256 * 256);

    long long cap;
    if (forced) {
        cap = forced_chunk;
    } else {
        gpu::MemInfo mi = gpu::mem_info();
        if (!mi.ok) {
            st.reason = "cudaMemGetInfo failed: " + mi.error;
            return false;
        }
        long long cs = compute::chunk_size(static_cast<long long>(mi.free_bytes),
                                           static_cast<long long>(mi.total_bytes),
                                           static_cast<long long>(kTabCap + kItabCap),
                                           static_cast<long long>((dof + 12) * sT));
        if (cs == 0) {
            st.reason = "not enough free VRAM";
            return false;
        }
        cap = std::min(cs, kMaxLaunchSamples);
    }
    cap = std::min(cap, n_round);

    try {
        for (;;) {
            cudaError_t e = ensure_pool(dof * static_cast<size_t>(cap) * sT,
                                        12 * static_cast<size_t>(cap) * sT);
            if (e == cudaSuccess) break;
            cap = (cap / 2) / 256 * 256;  // OOM is not sticky: halve and retry
            if (cap < min_cap || cap < 256) {
                st.reason = "out of GPU memory";
                return false;
            }
        }
        st.chunk = cap;

        std::vector<char> tab;
        std::vector<int> itab;
        gpu::fk_pack_tables(tables, fp64, tab, itab);
        CUDA_CHECK(cudaMemcpy(g_pool.d_tab, tab.data(), tab.size(), cudaMemcpyHostToDevice));
        if (!itab.empty())
            CUDA_CHECK(cudaMemcpy(g_pool.d_itab, itab.data(), itab.size() * sizeof(int),
                                  cudaMemcpyHostToDevice));

        Pending pend[2];
        long long cur = cap;
        auto finish = [&](int si) {
            Pending& pd = pend[si];
            Slot& sl = g_pool.slot[si];
            CUDA_CHECK(cudaStreamSynchronize(sl.st));
            float ms = 0.f;
            CUDA_CHECK(cudaEventElapsedTime(&ms, sl.e0, sl.e1));
            st.kernel_ms += ms;
            if (fp64)
                unpack_out<double>(static_cast<const double*>(sl.h_out), pd.start, pd.cn, pos, rot,
                                   world, Mb, nth);
            else
                unpack_out<float>(static_cast<const float*>(sl.h_out), pd.start, pd.cn, pos, rot,
                                  world, Mb, nth);
            pd.valid = false;
            if (!forced && ms > 0.f) {  // keep each launch near the 50 ms target
                long long nc = static_cast<long long>(static_cast<double>(pd.cn) *
                                                      (kTargetKernelMs / ms));
                nc = std::min(nc, cap);
                nc = std::max(nc, std::min(cap, 65536LL));
                nc -= nc % 256;
                cur = std::max(nc, 256LL);
            }
        };

        size_t pos_i = 0;
        int s = 0;
        while (pos_i < N) {
            if (pend[s].valid) finish(s);
            Slot& sl = g_pool.slot[s];
            const size_t cn = std::min<size_t>(static_cast<size_t>(cur), N - pos_i);
            if (fp64)
                pack_q<double>(static_cast<double*>(sl.h_in), q, pos_i, cn, dof, nth);
            else
                pack_q<float>(static_cast<float*>(sl.h_in), q, pos_i, cn, dof, nth);
            if (dof > 0)
                CUDA_CHECK(cudaMemcpyAsync(sl.d_in, sl.h_in, dof * cn * sT, cudaMemcpyHostToDevice,
                                           sl.st));
            CUDA_CHECK(cudaEventRecord(sl.e0, sl.st));
            const int grid = static_cast<int>(std::min<long long>(
                (static_cast<long long>(cn) + gpu::FK_THREADS - 1) / gpu::FK_THREADS,
                8LL * sm_count));
            gpu::fk_launch(fp64, tables.n, g_pool.d_tab, g_pool.d_itab, sl.d_in,
                           static_cast<long long>(cn), sl.d_out, static_cast<long long>(cn),
                           static_cast<long long>(cn), grid, sl.st);
            CUDA_CHECK(cudaEventRecord(sl.e1, sl.st));
            CUDA_CHECK(cudaMemcpyAsync(sl.h_out, sl.d_out, 12 * cn * sT, cudaMemcpyDeviceToHost,
                                       sl.st));
            pend[s] = {pos_i, cn, true};
            pos_i += cn;
            ++st.chunks;
            if (st.chunks == 1) finish(s);  // time the first chunk before sizing the rest
            s ^= 1;
        }
        if (pend[s].valid) finish(s);  // older chunk first
        if (pend[s ^ 1].valid) finish(s ^ 1);
    } catch (...) {
        cudaDeviceSynchronize();
        cudaGetLastError();
        throw;
    }
    return true;
}

template <class T>
std::vector<T> get_vec(const py::dict& d, const char* key) {
    if (!d.contains(key))
        throw std::invalid_argument(std::string("arrays: missing key '") + key + "'");
    py::object o = d[key];
    auto arr = py::array_t<T, py::array::c_style | py::array::forcecast>::ensure(o);
    if (!arr) throw std::invalid_argument(std::string("arrays: bad value for '") + key + "'");
    return std::vector<T>(arr.data(), arr.data() + arr.size());
}

kin::CompiledArrays parse_arrays(const py::dict& d) {
    kin::CompiledArrays a;
    auto geti = [&](const char* k) {
        if (!d.contains(k))
            throw std::invalid_argument(std::string("arrays: missing key '") + k + "'");
        return d[k].cast<int>();
    };
    a.n_joints = geti("n_joints");
    a.n_links = geti("n_links");
    a.n_dof = geti("n_dof");
    a.tcp_link = geti("tcp_link");
    a.layout_version = geti("layout_version");
    a.joint_index = get_vec<int32_t>(d, "joint_index");
    a.parent_link = get_vec<int32_t>(d, "parent_link");
    a.child_link = get_vec<int32_t>(d, "child_link");
    a.type = get_vec<int32_t>(d, "type");
    a.q_index = get_vec<int32_t>(d, "q_index");
    a.src_q_index = get_vec<int32_t>(d, "src_q_index");
    a.pre = get_vec<double>(d, "pre");
    a.post = get_vec<double>(d, "post");
    a.axis = get_vec<double>(d, "axis");
    a.mult = get_vec<double>(d, "mult");
    a.offset = get_vec<double>(d, "offset");
    a.base = get_vec<double>(d, "base");
    a.tool = get_vec<double>(d, "tool");
    return a;
}

gpu::FkPathTables build_tables(const kin::CompiledModel& cm, const std::vector<int>& path) {
    gpu::FkPathTables t;
    t.n = static_cast<int>(path.size());
    auto put34 = [](std::vector<double>& out, const Eigen::Isometry3d& T) {
        const Eigen::Matrix4d M = T.matrix();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c) out.push_back(M(r, c));
    };
    for (int idx : path) {
        const kin::CompiledJoint& j = cm.joints[idx];
        t.type.push_back(static_cast<int>(j.type));
        t.src_q.push_back(j.src_q_index);
        t.mult.push_back(j.mult);
        t.offset.push_back(j.offset);
        put34(t.pre, j.pre);
        put34(t.post, j.post);
        t.axis.push_back(j.axis[0]);
        t.axis.push_back(j.axis[1]);
        t.axis.push_back(j.axis[2]);
    }
    put34(t.tool, cm.tool);
    return t;
}

// Default FK precision: set by the R5.2 decision rule (docs/gpu-precision-decision.md).
constexpr const char* kDefaultFkPrecision = "fp32";

py::dict fk_batch_py(const py::dict& arrays,
                     py::array_t<double, py::array::c_style | py::array::forcecast> q,
                     const std::string& precision, const std::string& frame, long long chunk,
                     int n_threads) {
    if (precision != "fp32" && precision != "fp64")
        throw std::invalid_argument("fk_batch: precision must be 'fp32' or 'fp64'");
    if (frame != "world" && frame != "base")
        throw std::invalid_argument("fk_batch: frame must be 'world' or 'base'");
    if (chunk < 0 || (chunk > 0 && chunk % 256 != 0))
        throw std::invalid_argument("fk_batch: chunk must be 0 or a positive multiple of 256");

    kin::CompiledArrays a = parse_arrays(arrays);
    if (a.n_joints > gpu::FK_MAX_JOINTS || a.n_links > gpu::FK_MAX_JOINTS ||
        a.n_dof > gpu::FK_MAX_JOINTS)
        throw std::invalid_argument("fk_batch: more than 64 joints/links/dof is not supported");
    kin::CompiledModel cm = kin::from_arrays(a);
    if (q.ndim() != 2 || q.shape(1) != cm.n_dof)
        throw std::invalid_argument("fk_batch: q must have shape (N, " + std::to_string(cm.n_dof) +
                                    ")");
    const std::vector<int> path = kin::active_path(cm);
    const gpu::FkPathTables tables = build_tables(cm, path);

    const size_t N = static_cast<size_t>(q.shape(0));
    const bool world = (frame == "world");
    const bool fp64 = (precision == "fp64");
    py::array_t<double> pos_a(std::vector<py::ssize_t>{static_cast<py::ssize_t>(N), 3});
    py::array_t<double> rot_a(std::vector<py::ssize_t>{static_cast<py::ssize_t>(N), 3, 3});
    double Mb[16];
    {
        const Eigen::Matrix4d M = cm.base.matrix();
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) Mb[r * 4 + c] = M(r, c);
    }
    const double* qp = q.data();
    double* pp = pos_a.mutable_data();
    double* rp = rot_a.mutable_data();

    RunStats st;
    bool used_gpu = false;
    double total_ms = 0.0;
    if (N > 0) {
        py::gil_scoped_release nogil;
        std::lock_guard<std::mutex> lk(g_fk_mutex);
        const auto t0 = std::chrono::steady_clock::now();
        used_gpu = run_gpu(tables, fp64, qp, N, cm.n_dof, pp, rp, world, Mb, chunk, n_threads, st);
        if (!used_gpu) {  // CPU double fallback
            const size_t dof = static_cast<size_t>(cm.n_dof);
            std::vector<double> soa(dof * N);
            compute::parallel_for(N, n_threads, 8192, [&](size_t b, size_t e) {
                for (size_t i = b; i < e; ++i)
                    for (size_t d = 0; d < dof; ++d) soa[d * N + i] = qp[i * dof + d];
            });
            kin::FkBatchResult r = kin::fk_batch_cpu(cm, soa.data(), N, N, n_threads);
            compute::parallel_for(N, n_threads, 8192, [&](size_t b, size_t e) {
                for (size_t i = b; i < e; ++i) {
                    double v[12];
                    for (int k = 0; k < 3; ++k) v[k] = r.position[k * N + i];
                    for (int k = 0; k < 9; ++k) v[3 + k] = r.rotation[k * N + i];
                    double* po = pp + i * 3;
                    double* ro = rp + i * 9;
                    if (world) {
                        for (int a2 = 0; a2 < 3; ++a2) {
                            po[a2] = Mb[a2 * 4] * v[0] + Mb[a2 * 4 + 1] * v[1] +
                                     Mb[a2 * 4 + 2] * v[2] + Mb[a2 * 4 + 3];
                            for (int c = 0; c < 3; ++c)
                                ro[a2 * 3 + c] = Mb[a2 * 4] * v[3 + c] + Mb[a2 * 4 + 1] * v[6 + c] +
                                                 Mb[a2 * 4 + 2] * v[9 + c];
                        }
                    } else {
                        for (int k = 0; k < 3; ++k) po[k] = v[k];
                        for (int k = 0; k < 9; ++k) ro[k] = v[3 + k];
                    }
                }
            });
        }
        total_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
                       .count();
    } else {
        st.reason = "empty input";
    }

    py::dict info;
    info["precision"] = (N > 0 && !used_gpu) ? std::string("fp64") : precision;
    info["backend"] = used_gpu ? "cuda" : "cpu";
    info["chunks"] = st.chunks;
    info["chunk"] = st.chunk;
    info["kernel_ms"] = st.kernel_ms;
    info["total_ms"] = total_ms;
    info["fallback_reason"] = st.reason;
    py::dict out;
    out["position"] = pos_a;
    out["rotation"] = rot_a;
    out["info"] = info;
    return out;
}

// ---------------------------------------------------------------------------
// R5.3 workspace sampling (GPU, CPU fallback with the same result dict)
// ---------------------------------------------------------------------------

std::mutex g_ws_mutex;

py::dict sample_workspace_py(const py::dict& arrays, const std::vector<double>& lo,
                             const std::vector<double>& hi, const py::object& options,
                             const py::object& cancel, const py::object& progress) {
    kin::CompiledArrays a = parse_arrays(arrays);
    if (a.n_joints > gpu::FK_MAX_JOINTS || a.n_links > gpu::FK_MAX_JOINTS ||
        a.n_dof > gpu::FK_MAX_JOINTS)
        throw std::invalid_argument(
            "sample_workspace: more than 64 joints/links/dof is not supported");
    kin::CompiledModel cm = kin::from_arrays(a);
    compute::WorkspaceOptions o = compute::parse_workspace_options(options);
    const compute::WorkspaceGrid grid = compute::resolve_grid(cm, lo, hi, o);  // ValueError
    const compute::SamplerTables smp = compute::make_sampler(lo, hi, o);
    const std::size_t total = compute::total_samples(o, cm.n_dof);
    const gpu::FkPathTables tables = build_tables(cm, kin::active_path(cm));
    compute::RunControl rc(cancel, progress);

    compute::WorkspaceResult r;
    {
        py::gil_scoped_release nogil;
        std::lock_guard<std::mutex> lk(g_ws_mutex);
        std::string reason;
        const bool ok = gpu::workspace_gpu(tables, smp, grid, o, total, rc.flag(),
                                           rc.progress_fn(), r, reason);
        if (!ok) {  // CPU reference with the same Philox samples
            r = compute::sample_workspace_cpu(cm, lo, hi, o, rc.flag(), rc.progress_fn());
            r.fallback_reason = reason;
        }
    }
    rc.rethrow_if_failed();
    return compute::workspace_result_to_dict(r, o);
}

// ---------------------------------------------------------------------------
// R5.4 part B: GPU batch IK PROTOTYPE (GPU_IK_EXPERIMENTAL; not used by the backend).
// GPU fp32 DLS + CPU fp64 polish. CPU fallback (no polish) when the GPU cannot run.
// ---------------------------------------------------------------------------

std::mutex g_ik_mutex;
constexpr bool kGpuIkExperimental = true;

py::dict solve_ik_batch_py(const py::dict& arrays, const std::vector<double>& lo,
                           const std::vector<double>& hi, kin::DArr targets, kin::DArr q0,
                           const py::object& options, long long chunk, int n_threads) {
    kin::CompiledArrays a = parse_arrays(arrays);
    if (a.n_joints > gpu::FK_MAX_JOINTS || a.n_links > gpu::FK_MAX_JOINTS ||
        a.n_dof > gpu::FK_MAX_JOINTS)
        throw std::invalid_argument("solve_ik_batch: more than 64 joints/links/dof is not supported");
    if (a.n_dof > gpu::IK_MAX_DOF)
        throw std::invalid_argument("solve_ik_batch: GPU IK supports at most 12 DOF");
    kin::CompiledModel cm = kin::from_arrays(a);
    const kin::IKOptions opt = kin::parse_ik_options(options);
    if (opt.null_gain != 0.0)
        throw std::invalid_argument("solve_ik_batch: null_gain is not supported by the GPU IK");
    if (!(opt.weights[0] == opt.weights[1] && opt.weights[1] == opt.weights[2] &&
          opt.weights[3] == opt.weights[4] && opt.weights[4] == opt.weights[5]))
        throw std::invalid_argument(
            "solve_ik_batch: GPU IK needs weights uniform inside the position and rotation triples");
    if (chunk < 0 || (chunk > 0 && chunk % 128 != 0))
        throw std::invalid_argument("solve_ik_batch: chunk must be 0 or a positive multiple of 128");
    bool bc = false;
    const std::size_t N = kin::check_ik_batch_inputs(targets, q0, cm.n_dof, bc, "solve_ik_batch");
    if (lo.size() != static_cast<std::size_t>(cm.n_dof) || hi.size() != lo.size())
        throw std::invalid_argument("solve_ik_batch: lo/hi must have n_dof entries");

    const std::vector<int> path = kin::active_path(cm);
    const gpu::FkPathTables tables = build_tables(cm, path);

    gpu::IkGpuIn in;
    in.tables = &tables;
    gpu::IkParams& P = in.prm;
    P.nd = cm.n_dof;
    P.max_iter = opt.max_iter;
    const float wr = opt.position_only ? 0.f : 1.f;
    for (int i = 0; i < 6; ++i)
        P.w[i] = static_cast<float>(opt.weights[i]) * (i >= 3 ? wr : 1.f);
    P.tol_pos = static_cast<float>(std::max(opt.tol_pos, 1e-4));  // fp32 tolerance
    P.tol_rot = static_cast<float>(std::max(opt.tol_rot, 1e-4));
    P.lam_min = 5e-3f;
    P.lam0 = static_cast<float>(std::max(opt.damping, 5e-3));
    for (int d = 0; d < cm.n_dof; ++d) {
        P.lo[d] = opt.enforce_limits ? static_cast<float>(lo[d]) : -3e38f;
        P.hi[d] = opt.enforce_limits ? static_cast<float>(hi[d]) : 3e38f;
        P.cap[d] = 0.2f;
    }
    for (int j = 0; j < tables.n; ++j)
        if (tables.type[j] == 1 && tables.src_q[j] >= 0) P.cap[tables.src_q[j]] = 0.05f;
    in.targets = targets.data();
    in.q0 = q0.data();
    in.q0_broadcast = bc;
    in.N = N;
    in.forced_chunk = chunk;
    in.n_threads = n_threads;
    {
        const Eigen::Matrix4d M = cm.base.matrix();
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) in.base[r * 4 + c] = M(r, c);
    }

    kin::IkBatchResult res;
    gpu::IkGpuOut go;
    std::string reason;
    bool used_gpu = false;
    double gpu_ms = 0.0, polish_ms = 0.0;
    long long n_gpu_conv = 0;
    {
        py::gil_scoped_release nogil;
        std::lock_guard<std::mutex> lk(g_ik_mutex);
        const auto t0 = std::chrono::steady_clock::now();
        used_gpu = N > 0 && gpu::ik_gpu_run(in, go, reason);
        const auto t1 = std::chrono::steady_clock::now();
        if (used_gpu) {
            gpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            for (std::size_t i = 0; i < N; ++i)
                if (go.status[i] == gpu::IKS_CONVERGED) ++n_gpu_conv;
            kin::IKOptions po = opt;  // CPU fp64 polish from the GPU result
            po.max_iter = 10;
            po.damping = 1e-6;
            po.null_gain = 0.0;
            res = kin::ik_batch_cpu(cm, lo, hi, targets.data(), N, go.q.data(), false, po,
                                    n_threads);
            polish_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t1).count();
        } else {  // CPU double fallback (full solve, no polish)
            if (N == 0) reason = "empty input";
            res = kin::ik_batch_cpu(cm, lo, hi, targets.data(), N, q0.data(), bc, opt, n_threads);
            polish_ms = 0.0;
            gpu_ms = 0.0;
        }
    }
    long long n_conv = 0;
    for (std::size_t i = 0; i < N; ++i)
        if (res.status[i] == kin::IK_CONVERGED) ++n_conv;

    py::dict out = kin::ik_batch_result_to_dict(res);
    py::dict info;
    info["backend"] = used_gpu ? "cuda" : "cpu";
    info["gpu_ms"] = gpu_ms;
    info["kernel_ms"] = go.kernel_ms;
    info["polish_ms"] = polish_ms;
    info["n_gpu_converged"] = n_gpu_conv;
    info["n_polish_converged"] = n_conv;
    info["chunks"] = go.chunks;
    info["chunk"] = go.chunk;
    info["fallback_reason"] = reason;
    out["info"] = info;
    return out;
}

}  // namespace

PYBIND11_MODULE(gpu_bridge, m) {
    m.doc() = "Robotics Analyzer GPU bridge (CUDA, optional)";

    m.def("device_info", [] {
        gpu::DeviceInfo d = gpu::query_device();
        py::dict r;
        r["available"] = d.available;
        r["name"] = d.name;
        r["sm_major"] = d.sm_major;
        r["sm_minor"] = d.sm_minor;
        r["total_bytes"] = static_cast<unsigned long long>(d.total_bytes);
        r["free_bytes"] = static_cast<unsigned long long>(d.free_bytes);
        r["driver_version"] = d.driver_version;
        r["runtime_version"] = d.runtime_version;
        r["error"] = d.error;
        return r;
    }, "Return a dict describing CUDA device 0. Never raises.");

    m.def("mem_info", [] {
        gpu::MemInfo mi = gpu::mem_info();
        py::dict r;
        r["ok"] = mi.ok;
        r["free_bytes"] = static_cast<unsigned long long>(mi.free_bytes);
        r["total_bytes"] = static_cast<unsigned long long>(mi.total_bytes);
        r["error"] = mi.error;
        return r;
    }, "Return free/total VRAM in bytes. Never raises.");

    m.def("is_dead", [] { return gpu::is_dead(); },
          "True after a sticky CUDA error marked the GPU dead.");

    m.def("set_fake_oom", [](int n) { gpu::set_fake_oom(n); }, py::arg("n"),
          "Test hook: the next n GPU allocations report out-of-memory.");

    m.attr("DEFAULT_FK_PRECISION") = kDefaultFkPrecision;
    m.def("fk_batch", &fk_batch_py, py::arg("arrays"), py::arg("q"),
          py::arg("precision") = std::string(kDefaultFkPrecision), py::arg("frame") = "world",
          py::arg("chunk") = 0, py::arg("n_threads") = 0,
          "Batch TCP FK on the GPU (CPU double fallback). arrays = core_bridge.compile_arrays(model), "
          "q (N, n_dof). Returns dict position (N,3), rotation (N,3,3), info. chunk (multiple of 256) "
          "is a test hook that forces the launch size.");
    compute::register_cancel_token(m);
    m.def("sample_workspace", &sample_workspace_py, py::arg("arrays"), py::arg("lo"), py::arg("hi"),
          py::arg("options"), py::arg("cancel") = py::none(), py::arg("progress") = py::none(),
          "Workspace sampling on the GPU (fp32, Philox q on the device); CPU reference fallback. "
          "Same result dict as core_bridge.sample_workspace.");
    m.def("sample_q", [](const std::vector<double>& lo, const std::vector<double>& hi,
                         const py::object& options, long long start, long long n) {
        compute::WorkspaceOptions o = compute::parse_workspace_options(options);
        if (lo.size() != hi.size() || lo.size() > 64)
            throw std::invalid_argument("sample_q: lo/hi size mismatch or more than 64 DOF");
        if (start < 0 || n < 0) throw std::invalid_argument("sample_q: start and n must be >= 0");
        if (o.mode != "random" && o.mode != "grid")
            throw std::invalid_argument("sample_q: mode must be 'random' or 'grid'");
        if (o.mode == "grid" && o.divisions.size() != lo.size())
            throw std::invalid_argument("sample_q: divisions must have n_dof entries");
        for (int dv : o.divisions)
            if (dv < 1) throw std::invalid_argument("sample_q: every division must be >= 1");
        const compute::SamplerTables s = compute::make_sampler(lo, hi, o);
        py::array_t<float> out(std::vector<py::ssize_t>{static_cast<py::ssize_t>(n),
                                                        static_cast<py::ssize_t>(lo.size())});
        {
            py::gil_scoped_release nogil;
            std::lock_guard<std::mutex> lk(g_ws_mutex);
            gpu::sample_q_gpu(s, static_cast<std::size_t>(start), static_cast<std::size_t>(n),
                              out.mutable_data());
        }
        return out;
    }, py::arg("lo"), py::arg("hi"), py::arg("options") = py::none(), py::arg("start") = 0,
       py::arg("n") = 0, "Debug: float32 q (n, n_dof) generated on the device (same as core_bridge.sample_q).");
    m.attr("GPU_IK_EXPERIMENTAL") = kGpuIkExperimental;
    m.def("solve_ik_batch", &solve_ik_batch_py, py::arg("arrays"), py::arg("lo"), py::arg("hi"),
          py::arg("targets"), py::arg("q0"), py::arg("options") = py::none(),
          py::arg("chunk") = 0, py::arg("n_threads") = 0,
          "EXPERIMENTAL (GPU_IK_EXPERIMENTAL, not used by the backend): fp32 DLS IK on the GPU + "
          "CPU fp64 polish (10 iterations). Same result dict as core_bridge.solve_ik_batch plus "
          "info (gpu_ms, polish_ms, n_polish_converged, ...). CPU fallback without a GPU. chunk "
          "(multiple of 128) is a test hook.");
    m.def("release_buffers", [] {
        std::lock_guard<std::mutex> lk(g_fk_mutex);
        cudaDeviceSynchronize();
        pool_free_buffers();
    }, "Free the pooled pinned/device FK buffers.");
}
