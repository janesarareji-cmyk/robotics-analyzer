/**
 * ik_py.hpp
 * R5.4: pybind11 helpers shared by core_bridge (CPU) and gpu_bridge (CUDA):
 * IK options dict parsing, batch input checks, batch result dict.
 * Header only; include from binding files only.
 */
#pragma once

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "kin/ik.hpp"
#include "kin/ik_batch.hpp"

namespace kin {

namespace py = pybind11;

inline IKOptions parse_ik_options(const py::object& options) {
    IKOptions o;
    if (options.is_none()) return o;
    for (auto item : options.cast<py::dict>()) {
        const std::string k = py::str(item.first);
        py::handle v = item.second;
        if (k == "position_only") o.position_only = v.cast<bool>();
        else if (k == "weights") {
            auto w = v.cast<std::vector<double>>();
            if (w.size() != 6) throw std::invalid_argument("solve_ik: weights needs 6 values");
            std::copy(w.begin(), w.end(), o.weights.begin());
        }
        else if (k == "damping") o.damping = v.cast<double>();
        else if (k == "max_iter") o.max_iter = v.cast<int>();
        else if (k == "tol_pos") o.tol_pos = v.cast<double>();
        else if (k == "tol_rot") o.tol_rot = v.cast<double>();
        else if (k == "max_step") o.max_step = v.cast<double>();
        else if (k == "enforce_limits") o.enforce_limits = v.cast<bool>();
        else if (k == "null_gain") o.null_gain = v.cast<double>();
        else throw std::invalid_argument("solve_ik: unknown option '" + k + "'");
    }
    // A1.4 (audit LOW): reject values that make the solver misbehave.
    if (!(o.damping > 0.0) || !std::isfinite(o.damping))
        throw std::invalid_argument("solve_ik: damping must be > 0");
    if (o.max_iter < 1 || o.max_iter > 100000)
        throw std::invalid_argument("solve_ik: max_iter must be in [1, 100000]");
    if (!(o.max_step > 0.0) || !std::isfinite(o.max_step))
        throw std::invalid_argument("solve_ik: max_step must be > 0");
    if (!(o.tol_pos > 0.0) || !std::isfinite(o.tol_pos) || !(o.tol_rot > 0.0) || !std::isfinite(o.tol_rot))
        throw std::invalid_argument("solve_ik: tol_pos and tol_rot must be > 0");
    return o;
}

using DArr = py::array_t<double, py::array::c_style | py::array::forcecast>;

/// Validates targets (N,4,4) and q0 (N,dof) or (dof,); returns N, sets broadcast.
inline std::size_t check_ik_batch_inputs(const DArr& targets, const DArr& q0, int n_dof,
                                         bool& broadcast, const char* who) {
    const std::string w(who);
    if (targets.ndim() != 3 || targets.shape(1) != 4 || targets.shape(2) != 4)
        throw std::invalid_argument(w + ": targets must have shape (N, 4, 4)");
    const std::size_t N = static_cast<std::size_t>(targets.shape(0));
    broadcast = false;
    if (q0.ndim() == 1 && q0.shape(0) == n_dof) {
        broadcast = true;
    } else if (q0.ndim() == 2 && q0.shape(1) == n_dof &&
               static_cast<std::size_t>(q0.shape(0)) == N) {
        broadcast = false;
    } else {
        throw std::invalid_argument(w + ": q0 must have shape (" + std::to_string(n_dof) +
                                    ",) or (N, " + std::to_string(n_dof) + ")");
    }
    return N;
}

inline py::dict ik_batch_result_to_dict(const IkBatchResult& r) {
    const py::ssize_t n = static_cast<py::ssize_t>(r.n);
    py::array_t<double> q(std::vector<py::ssize_t>{n, static_cast<py::ssize_t>(r.n_dof)});
    py::array_t<int> st(std::vector<py::ssize_t>{n});
    py::array_t<double> pe(std::vector<py::ssize_t>{n}), re(std::vector<py::ssize_t>{n});
    py::array_t<int> it(std::vector<py::ssize_t>{n});
    py::array_t<bool> cv(std::vector<py::ssize_t>{n});
    std::copy(r.q.begin(), r.q.end(), q.mutable_data());
    std::copy(r.status.begin(), r.status.end(), st.mutable_data());
    std::copy(r.pos_error.begin(), r.pos_error.end(), pe.mutable_data());
    std::copy(r.rot_error.begin(), r.rot_error.end(), re.mutable_data());
    std::copy(r.iterations.begin(), r.iterations.end(), it.mutable_data());
    bool* c = cv.mutable_data();
    for (std::size_t i = 0; i < r.n; ++i) c[i] = (r.status[i] == IK_CONVERGED);
    py::dict d;
    d["q"] = q;
    d["status"] = st;
    d["converged"] = cv;
    d["pos_error"] = pe;
    d["rot_error"] = re;
    d["iterations"] = it;
    return d;
}

}  // namespace kin
