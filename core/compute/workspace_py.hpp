/**
 * workspace_py.hpp
 * R5.3: pybind11 helpers shared by core_bridge (CPU) and gpu_bridge (CUDA):
 * options dict parsing, result dict, CancelToken, progress callback wrapper.
 * Header only; include from binding files only.
 */
#pragma once

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <atomic>
#include <cstring>
#include <set>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include "compute/workspace.hpp"

namespace compute {

namespace py = pybind11;

/// Cooperative cancel flag shared with Python (`CancelToken().cancel()`).
struct CancelToken {
    std::atomic<bool> flag{false};
    void cancel() { flag.store(true, std::memory_order_relaxed); }
    void reset() { flag.store(false, std::memory_order_relaxed); }
    bool is_cancelled() const { return flag.load(std::memory_order_relaxed); }
};

/// Registers CancelToken once per process; the second module reuses the first type.
inline void register_cancel_token(py::module_& m) {
    if (py::detail::get_type_info(typeid(CancelToken))) {
        m.attr("CancelToken") = py::type::of<CancelToken>();
        return;
    }
    py::class_<CancelToken>(m, "CancelToken",
                            "Cancel flag (std::atomic<bool>) checked between chunks.")
        .def(py::init<>())
        .def("cancel", &CancelToken::cancel)
        .def("reset", &CancelToken::reset)
        .def("is_cancelled", &CancelToken::is_cancelled);
}

inline std::vector<double> to_vec3(const py::handle& h, const char* what) {
    std::vector<double> v = h.cast<std::vector<double>>();
    if (v.size() != 3) throw std::invalid_argument(std::string(what) + " must have 3 entries");
    return v;
}

inline WorkspaceOptions parse_workspace_options(const py::object& options) {
    WorkspaceOptions o;
    if (options.is_none()) return o;
    if (!py::isinstance<py::dict>(options))
        throw std::invalid_argument("options must be a dict");
    static const std::set<std::string> known = {
        "mode",       "n_samples",  "seed",      "voxel_size", "bounds_lo",
        "bounds_hi",  "want_counts", "want_points", "max_points", "budget_bytes",
        "divisions",  "chunk",      "n_threads"};
    py::dict d = options.cast<py::dict>();
    for (auto item : d) {
        const std::string k = py::str(item.first);
        if (!known.count(k)) throw std::invalid_argument("unknown option '" + k + "'");
    }
    auto has = [&](const char* k) { return d.contains(k) && !d[k].is_none(); };
    if (has("mode")) o.mode = d["mode"].cast<std::string>();
    if (has("n_samples")) {
        const long long v = d["n_samples"].cast<long long>();
        if (v < 0) throw std::invalid_argument("n_samples must be >= 0");
        o.n_samples = static_cast<std::size_t>(v);
    }
    if (has("seed")) o.seed = d["seed"].cast<std::uint64_t>();
    if (has("voxel_size")) o.voxel_size = d["voxel_size"].cast<double>();
    if (has("bounds_lo") != has("bounds_hi"))
        throw std::invalid_argument("bounds_lo and bounds_hi must be given together");
    if (has("bounds_lo")) {
        const auto lo = to_vec3(d["bounds_lo"], "bounds_lo");
        const auto hi = to_vec3(d["bounds_hi"], "bounds_hi");
        for (int c = 0; c < 3; ++c) {
            o.bounds_lo[c] = lo[c];
            o.bounds_hi[c] = hi[c];
        }
        o.bounds_set = true;
    }
    if (has("want_counts")) o.want_counts = d["want_counts"].cast<bool>();
    if (has("want_points")) o.want_points = d["want_points"].cast<bool>();
    if (has("max_points")) {
        const long long v = d["max_points"].cast<long long>();
        if (v < 0) throw std::invalid_argument("max_points must be >= 0");
        o.max_points = static_cast<std::size_t>(v);
    }
    if (has("budget_bytes")) {
        const long long v = d["budget_bytes"].cast<long long>();
        if (v < 0) throw std::invalid_argument("budget_bytes must be >= 0");
        o.budget_bytes = static_cast<std::size_t>(v);
    }
    if (has("divisions")) o.divisions = d["divisions"].cast<std::vector<int>>();
    if (has("chunk")) o.chunk = d["chunk"].cast<long long>();
    if (has("n_threads")) o.n_threads = d["n_threads"].cast<int>();
    return o;
}

template <class T>
py::array_t<T> vec_to_np(const std::vector<T>& v, std::vector<py::ssize_t> shape) {
    py::array_t<T> a(shape);
    if (!v.empty()) std::memcpy(a.mutable_data(), v.data(), v.size() * sizeof(T));
    return a;
}

inline py::dict workspace_result_to_dict(const WorkspaceResult& r, const WorkspaceOptions& o) {
    py::dict d;
    const py::ssize_t nx = r.grid.dims[0], ny = r.grid.dims[1], nz = r.grid.dims[2];
    d["occupied"] = vec_to_np(r.occupied, {static_cast<py::ssize_t>(r.occupied.size())});
    if (o.want_counts)
        d["counts"] = vec_to_np(r.counts, {nx, ny, nz});
    else
        d["counts"] = py::none();
    d["occupied_voxels"] =
        vec_to_np(r.voxels, {static_cast<py::ssize_t>(r.voxels.size() / 3), 3});
    if (o.want_points)
        d["points"] = vec_to_np(r.points, {static_cast<py::ssize_t>(r.points.size() / 3), 3});
    else
        d["points"] = py::none();
    d["grid_dims"] = py::make_tuple(r.grid.dims[0], r.grid.dims[1], r.grid.dims[2]);
    d["origin"] = py::make_tuple(r.grid.origin[0], r.grid.origin[1], r.grid.origin[2]);
    d["voxel_size"] = r.grid.voxel;
    d["n_samples"] = static_cast<unsigned long long>(r.n_samples);
    d["n_samples_done"] = static_cast<unsigned long long>(r.n_done);
    d["n_outside"] = static_cast<unsigned long long>(r.n_outside);
    d["cancelled"] = r.cancelled;
    py::dict info;
    info["backend"] = r.backend;
    info["fallback_reason"] = r.fallback_reason;
    info["chunks"] = r.chunks;
    info["chunk"] = r.chunk;
    info["vram_peak_bytes"] = static_cast<unsigned long long>(r.vram_peak_bytes);
    info["budget_bytes"] = static_cast<unsigned long long>(r.budget_bytes);
    info["kernel_ms"] = r.kernel_ms;
    info["total_ms"] = r.total_ms;
    info["gpu_compaction"] = r.gpu_compaction;
    d["info"] = info;
    return d;
}

/// Cancel flag + progress callback for one run. Call from the GIL-released region only
/// through `progress_fn()`; the GIL is taken just for the Python call.
struct RunControl {
    CancelToken* token = nullptr;
    CancelToken local;
    py::object progress;
    std::string error;  // set when the Python progress callback raised

    RunControl(const py::object& cancel, const py::object& prog) : progress(prog) {
        if (!cancel.is_none()) {
            if (!py::isinstance<CancelToken>(cancel))
                throw std::invalid_argument("cancel must be a CancelToken or None");
            token = cancel.cast<CancelToken*>();
        }
    }
    std::atomic<bool>* flag() { return token ? &token->flag : &local.flag; }
    ProgressFn progress_fn() {
        if (progress.is_none()) return nullptr;
        return [this](std::size_t done, std::size_t total) {
            py::gil_scoped_acquire gil;
            try {
                progress(static_cast<unsigned long long>(done),
                         static_cast<unsigned long long>(total));
            } catch (py::error_already_set& e) {
                if (error.empty()) error = e.what();
                flag()->store(true, std::memory_order_relaxed);
            }
        };
    }
    /// Call with the GIL held after the run.
    void rethrow_if_failed() const {
        if (!error.empty())
            throw std::runtime_error("progress callback raised: " + error);
    }
};

}  // namespace compute
