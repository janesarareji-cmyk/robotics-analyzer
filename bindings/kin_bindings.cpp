/**
 * bindings/kin_bindings.cpp
 * pybind11 bindings for forward kinematics (kin).
 * R2.5: Exposes fk(model), fk(model, q), and pose_to_iso(pose).
 */

#include <algorithm>
#include <array>
#include <cstring>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "kin/fk.hpp"
#include "kin/compiled_model.hpp"
#include "kin/compiled_arrays.hpp"
#include "kin/fk_batch.hpp"
#include "compute/parallel.hpp"
#include "compute/philox.hpp"
#include "compute/workspace.hpp"
#include "compute/workspace_py.hpp"
#include "kin/jacobian.hpp"
#include "kin/ik.hpp"
#include "kin/ik_multi.hpp"
#include "kin/singularity.hpp"
#include <stdexcept>
#include "urm/robot_model.hpp"

namespace py = pybind11;

static py::dict fk_result_to_dict(const kin::FKResult& res, const std::vector<int>& active = {}) {
    py::dict d;
    py::list frames_list;
    for (const auto& f : res.frames) {
        frames_list.append(Eigen::Matrix4d(f.matrix()));
    }
    d["frames"] = frames_list;
    d["tcp"] = Eigen::Matrix4d(res.tcp.matrix());
    d["active_joints"] = active;
    return d;
}

/// Copy a vector into a new numpy array of the given shape (copy, not view).
template <typename T>
static py::array_t<T> to_np(const std::vector<T>& v, std::vector<py::ssize_t> shape) {
    py::array_t<T> arr(shape);
    if (!v.empty()) {
        std::memcpy(arr.mutable_data(), v.data(), v.size() * sizeof(T));
    }
    return arr;
}

static py::dict compiled_arrays_to_dict(const kin::CompiledArrays& a) {
    const py::ssize_t n = a.n_joints;
    py::dict d;
    d["n_joints"] = a.n_joints;
    d["n_links"] = a.n_links;
    d["n_dof"] = a.n_dof;
    d["tcp_link"] = a.tcp_link;
    d["layout_version"] = a.layout_version;
    d["joint_index"] = to_np(a.joint_index, {n});
    d["parent_link"] = to_np(a.parent_link, {n});
    d["child_link"] = to_np(a.child_link, {n});
    d["type"] = to_np(a.type, {n});
    d["q_index"] = to_np(a.q_index, {n});
    d["src_q_index"] = to_np(a.src_q_index, {n});
    d["pre"] = to_np(a.pre, {n, 4, 4});
    d["post"] = to_np(a.post, {n, 4, 4});
    d["axis"] = to_np(a.axis, {n, 3});
    d["mult"] = to_np(a.mult, {n});
    d["offset"] = to_np(a.offset, {n});
    d["base"] = to_np(a.base, {4, 4});
    d["tool"] = to_np(a.tool, {4, 4});
    return d;
}

static kin::IKOptions parse_ik_options(const py::object& options) {

        kin::IKOptions o;

        if (!options.is_none()) {

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

        }

    return o;

}



void bind_kin(py::module_& m) {
    py::class_<kin::CompiledModel>(m, "CompiledModel")
        .def_readonly("n_links", &kin::CompiledModel::n_links)
        .def_readonly("n_dof", &kin::CompiledModel::n_dof)
        .def_readonly("tcp_link", &kin::CompiledModel::tcp_link);

    m.def("fk", [](const urm::RobotModel& model) {
        return fk_result_to_dict(kin::forward_kinematics(model), kin::active_chain(model));
    }, py::arg("model"),
       "Compute forward kinematics using joint q values stored in model.");

    m.def("fk", [](const urm::RobotModel& model, const std::vector<double>& q) {
        return fk_result_to_dict(kin::forward_kinematics(model, q), kin::active_chain(model));
    }, py::arg("model"), py::arg("q"),
       "Compute forward kinematics for an explicit joint value vector q (size dof()).");

    m.def("active_chain", [](const urm::RobotModel& model) {
        return kin::active_chain(model);
    }, py::arg("model"),
       "URM joint indices on the path link 0 -> tcp_link, root to tip.");

    m.def("compile", [](const urm::RobotModel& model) {
        return kin::compile(model);
    }, py::arg("model"),
       "Compile a model once for repeated FK (raises ValueError on bad model).");

    m.def("fk_compiled", [](const kin::CompiledModel& cm, const std::vector<double>& q) {
        std::vector<int> active;
        int cur = cm.tcp_link;
        while (cur != 0) {
            int found = -1;
            for (const auto& j : cm.joints) if (j.child_link == cur) { found = j.joint_index; cur = j.parent_link; break; }
            if (found < 0) break;
            active.push_back(found);
        }
        std::reverse(active.begin(), active.end());
        return fk_result_to_dict(kin::forward_kinematics(cm, q), active);
    }, py::arg("cm"), py::arg("q"),
       "FK on a compiled model; same dict as fk(model, q).");

    m.def("fk_batch", [](const kin::CompiledModel& cm,
                         py::array_t<double, py::array::c_style | py::array::forcecast> q,
                         const std::string& frame, int n_threads) {
        if (frame != "world" && frame != "base")
            throw std::invalid_argument("fk_batch: frame must be 'world' or 'base'");
        if (q.ndim() != 2 || q.shape(1) != cm.n_dof)
            throw std::invalid_argument("fk_batch: q must have shape (N, " +
                                        std::to_string(cm.n_dof) + ")");
        kin::active_path(cm);  // validates the tcp link before any work
        const std::size_t N = static_cast<std::size_t>(q.shape(0));
        const std::size_t dof = static_cast<std::size_t>(cm.n_dof);
        py::array_t<double> pos(std::vector<py::ssize_t>{static_cast<py::ssize_t>(N), 3});
        py::array_t<double> rot(std::vector<py::ssize_t>{static_cast<py::ssize_t>(N), 3, 3});
        const double* qp = q.data();
        double* pp = pos.mutable_data();
        double* rp = rot.mutable_data();
        const bool world = (frame == "world");
        const Eigen::Matrix4d Mb = cm.base.matrix();
        {
            py::gil_scoped_release nogil;
            std::vector<double> soa(dof * N);
            compute::parallel_for(N, n_threads, 8192, [&](std::size_t b, std::size_t e) {
                for (std::size_t i = b; i < e; ++i)
                    for (std::size_t d = 0; d < dof; ++d) soa[d * N + i] = qp[i * dof + d];
            });
            kin::FkBatchResult r = kin::fk_batch_cpu(cm, soa.data(), N, N, n_threads);
            compute::parallel_for(N, n_threads, 8192, [&](std::size_t b, std::size_t e) {
                for (std::size_t i = b; i < e; ++i) {
                    Eigen::Matrix3d R;
                    for (int a = 0; a < 3; ++a)
                        for (int c = 0; c < 3; ++c) R(a, c) = r.rotation[(a * 3 + c) * N + i];
                    Eigen::Vector3d p(r.position[i], r.position[N + i], r.position[2 * N + i]);
                    if (world) {
                        p = Mb.topLeftCorner<3, 3>() * p + Mb.topRightCorner<3, 1>();
                        R = Mb.topLeftCorner<3, 3>() * R;
                    }
                    for (int a = 0; a < 3; ++a) pp[i * 3 + a] = p[a];
                    for (int a = 0; a < 3; ++a)
                        for (int c = 0; c < 3; ++c) rp[i * 9 + a * 3 + c] = R(a, c);
                }
            });
        }
        py::dict d;
        d["position"] = pos;
        d["rotation"] = rot;
        return d;
    }, py::arg("cm"), py::arg("q"), py::arg("frame") = "world", py::arg("n_threads") = 0,
       "Batch TCP FK (double, multi-threaded). q (N, n_dof) -> dict position (N,3), rotation (N,3,3).");

    // R5.3: workspace sampling (CPU reference), Philox, CancelToken
    compute::register_cancel_token(m);
    m.def("philox4x32_10", [](const std::array<std::uint32_t, 4>& ctr,
                              const std::array<std::uint32_t, 2>& key) {
        std::uint32_t out[4];
        compute::philox4x32_10(ctr.data(), key.data(), out);
        return std::array<std::uint32_t, 4>{out[0], out[1], out[2], out[3]};
    }, py::arg("counter"), py::arg("key"), "Philox4x32-10 block: 4 counter words, 2 key words -> 4 words.");

    m.def("sample_q", [](const std::vector<double>& lo, const std::vector<double>& hi,
                         const py::object& options, long long start, long long n) {
        compute::WorkspaceOptions o = compute::parse_workspace_options(options);
        if (lo.size() != hi.size()) throw std::invalid_argument("sample_q: lo/hi size mismatch");
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
        compute::sample_q_host(s, static_cast<std::size_t>(start), static_cast<std::size_t>(n),
                               out.mutable_data());
        return out;
    }, py::arg("lo"), py::arg("hi"), py::arg("options") = py::none(), py::arg("start") = 0,
       py::arg("n") = 0,
       "Float32 q (n, n_dof) of the workspace sampler for global samples start..start+n "
       "(options: mode, seed, divisions).");

    m.def("sample_workspace", [](const kin::CompiledModel& cm, const std::vector<double>& lo,
                                 const std::vector<double>& hi, const py::object& options,
                                 const py::object& cancel, const py::object& progress) {
        compute::WorkspaceOptions o = compute::parse_workspace_options(options);
        compute::RunControl rc(cancel, progress);
        compute::WorkspaceResult r;
        {
            py::gil_scoped_release nogil;
            r = compute::sample_workspace_cpu(cm, lo, hi, o, rc.flag(), rc.progress_fn());
        }
        rc.rethrow_if_failed();
        return compute::workspace_result_to_dict(r, o);
    }, py::arg("cm"), py::arg("lo"), py::arg("hi"), py::arg("options"),
       py::arg("cancel") = py::none(), py::arg("progress") = py::none(),
       "Workspace sampling on the CPU (double FK, Philox float q). Returns dict: occupied "
       "(bitset words), counts (nx,ny,nz) or None, occupied_voxels (M,3), points (P,3) or None, "
       "grid_dims, origin, voxel_size, n_samples_done, n_outside, cancelled, info.");

    m.def("jacobian", [](const kin::CompiledModel& cm, const std::vector<double>& q, int link,
                         const std::array<double, 3>& point_local) {
        return kin::jacobian(cm, q, link, Eigen::Vector3d(point_local[0], point_local[1], point_local[2]));
    }, py::arg("cm"), py::arg("q"), py::arg("link"),
       py::arg("point_local") = std::array<double, 3>{0.0, 0.0, 0.0},
       "World-frame 6 x n_dof Jacobian of a point fixed in link `link` (rows: v, w).");

    m.def("jacobian_tcp", [](const kin::CompiledModel& cm, const std::vector<double>& q) {
        return kin::jacobian_tcp(cm, q);
    }, py::arg("cm"), py::arg("q"),
       "World-frame 6 x n_dof Jacobian at the TCP (angular part of tcp_link).");

    m.def("solve_ik", [](const kin::CompiledModel& cm, const std::vector<double>& lo,
                         const std::vector<double>& hi, const Eigen::Matrix4d& target,
                         const std::vector<double>& q0, const py::object& options) {
        kin::IKOptions o = parse_ik_options(options);
        Eigen::Isometry3d T(target);
        kin::IKResult r = kin::solve_ik(cm, lo, hi, T, q0, o);
        py::dict d;
        d["q"] = r.q;
        d["converged"] = r.converged;
        d["pos_error"] = r.pos_error;
        d["rot_error"] = r.rot_error;
        d["iterations"] = r.iterations;
        d["message"] = r.message;
        return d;
    }, py::arg("cm"), py::arg("lo"), py::arg("hi"), py::arg("target"), py::arg("q0"),
       py::arg("options") = py::none(),
       "Damped least squares IK to a 4x4 world TCP target. Returns dict.");

    m.def("solve_ik_multi", [](const kin::CompiledModel& cm, const std::vector<double>& lo,
                               const std::vector<double>& hi, const Eigen::Matrix4d& target,
                               const std::vector<double>& q_current, const py::object& options,
                               int n_starts, uint64_t seed, double dedupe_tol) {
        kin::IKOptions o = parse_ik_options(options);
        Eigen::Isometry3d T(target);
        auto sols = kin::solve_ik_multi(cm, lo, hi, T, q_current, o, n_starts, seed, dedupe_tol);
        py::list out;
        for (const auto& s : sols) {
            py::dict d;
            d["q"] = s.q;
            d["pos_error"] = s.pos_error;
            d["rot_error"] = s.rot_error;
            d["distance"] = s.distance;
            out.append(d);
        }
        return out;
    }, py::arg("cm"), py::arg("lo"), py::arg("hi"), py::arg("target"), py::arg("q_current"),
       py::arg("options") = py::none(), py::arg("n_starts") = 30, py::arg("seed") = 42,
       py::arg("dedupe_tol") = 1e-3,
       "Multi-start IK: distinct validated solutions sorted by distance to q_current.");

    m.def("analyze_singularity", [](const Eigen::MatrixXd& J, const std::string& mode, double threshold) {
        const kin::SingularityInfo s = kin::analyze_singularity(J, mode, threshold);
        py::dict d;
        d["singular_values"] = s.singular_values;
        d["sigma_min"] = s.sigma_min;
        d["sigma_max"] = s.sigma_max;
        d["condition"] = s.condition;
        d["manipulability"] = s.manipulability;
        d["near_singular"] = s.near_singular;
        return d;
    }, py::arg("J"), py::arg("mode") = "full", py::arg("threshold") = 1e-3,
       "Singularity indicators of a 6 x n Jacobian. mode: full | position | angular. Returns dict.");

    m.def("compile_arrays", [](const urm::RobotModel& model) {
        return compiled_arrays_to_dict(kin::to_arrays(kin::compile(model)));
    }, py::arg("model"),
       "Flat frozen-layout arrays of the compiled model (see docs/compiled-model-layout.md).");

    m.def("effective_links", [](const urm::RobotModel& model) {
        return kin::effective_links(model);
    }, py::arg("model"),
       "Effective (parent_link, child_link) per URM joint (legacy 0/0 serial rule applied).");

    m.def("pose_to_iso", [](const urm::Pose& pose) {
        return Eigen::Matrix4d(kin::pose_to_iso(pose).matrix());
    }, py::arg("pose"),
       "Convert a 6D urm.Pose to a 4x4 homogeneous transformation matrix.");
}

